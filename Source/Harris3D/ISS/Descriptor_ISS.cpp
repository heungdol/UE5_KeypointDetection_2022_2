#include "Descriptor_ISS.h"


bool IsLocalMaxima(int query_idx,
				   const std::vector<int>& indices,
				   const std::vector<double>& third_eigen_values) {
	for (const auto& idx : indices) {
		if (query_idx == idx)
			continue;
		
		if (third_eigen_values[query_idx] < third_eigen_values[idx]) {
			return false;
		}
	}
	return true;
}

double ComputeModelResolution(const std::vector<Eigen::Vector3d>& points,
							  const KDTreeFlann& kdtree) {
	std::vector<int> indices(2);
	std::vector<double> distances(2);
	double resolution = 0.0;

	for (const auto& point : points) {
		if (kdtree.SearchKNN(point, 2, indices, distances) != 0) {
			resolution += std::sqrt(distances[1]);
		}
	}
	resolution /= points.size();

	indices.clear();
	std::vector<int>().swap(indices);

	distances.clear();
	std::vector<double>().swap(distances);
	
	return resolution;
}

template <typename IdxType>
Eigen::Matrix3d ComputeCovariance(const std::vector<Eigen::Vector3d> &points,
								  const std::vector<IdxType> &indices)
{
	if (indices.empty()) {
		return Eigen::Matrix3d::Identity();
	}

	// TODO 매트릭스 메모리 누수 확인
	Eigen::Matrix3d covariance;
	Eigen::Matrix<double, 9, 1> cumulants;
	cumulants.setZero();
	for (const auto &idx : indices) {
		const Eigen::Vector3d &point = points[idx];
		cumulants(0) += point(0);
		cumulants(1) += point(1);
		cumulants(2) += point(2);
		cumulants(3) += point(0) * point(0);
		cumulants(4) += point(0) * point(1);
		cumulants(5) += point(0) * point(2);
		cumulants(6) += point(1) * point(1);
		cumulants(7) += point(1) * point(2);
		cumulants(8) += point(2) * point(2);
	}
	cumulants /= (double)indices.size();
	covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
	covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
	covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
	covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
	covariance(1, 0) = covariance(0, 1);
	covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
	covariance(2, 0) = covariance(0, 2);
	covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
	covariance(2, 1) = covariance(1, 2);

	return covariance;
}

std::vector<int> Descriptor_ISS::ComputeISSKeypoints(
	const std::vector<Eigen::Vector3d>& input, double salient_radius, double non_max_radius, double gamma_21,
	double gamma_32, int min_neighbors)
{
    if (input.empty())
    {
        //utility::LogWarning("[ComputeISSKeypoints] Input PointCloud is empty!");
        return std::vector<int>{};
    }
	
	Eigen::MatrixXd _input = Eigen::MatrixXd::Zero (3, input.size());
	// TODO vector vector3d -> vectorxd로 변환할 것
	for (int _i = 0; _i < input.size(); _i++)
	{
		// _input.setConstant(0, _i, input[_i].x());
		// _input.setConstant(1, _i, input[_i].y());
		// _input.setConstant(2, _i, input[_i].z());
		_input (0, _i) = input[_i][0];
		_input (1, _i) = input[_i][1];
		_input (2, _i) = input[_i][2];
	}
	
    if (!kdtree.SetMatrixData(_input))
    {
    	return std::vector<int>{};
    }
	
    if (salient_radius == 0.0 || non_max_radius == 0.0) {
        const double resolution = ComputeModelResolution(input, kdtree);
        salient_radius = 6 * resolution;
        non_max_radius = 4 * resolution;
        // utility::LogDebug(
        //         "[ComputeISSKeypoints] Computed salient_radius = {}, "
        //         "non_max_radius = {} from input model",
        //         salient_radius, non_max_radius);
    }
	eigenValues.clear();
	vector<double>().swap(eigenValues);

//#pragma omp parallel for schedule(static) shared(third_eigen_values)
    for (int i = 0; i < (int)input.size(); i++) {
        std::vector<int> indices;
        std::vector<double> dist;
        int nb_neighbors =
                kdtree.SearchRadius(input[i], salient_radius, indices, dist);
        if (nb_neighbors < min_neighbors) {
        	eigenValues.push_back(0);
            continue;
        }

        Eigen::Matrix3d cov = ComputeCovariance(input, indices);
        if (cov.isZero()) {
        	eigenValues.push_back(0);
            continue;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        double e1c = solver.eigenvalues()[2];
        double e2c = solver.eigenvalues()[1];
        double e3c = solver.eigenvalues()[0];

        if ((e2c / e1c) < gamma_21 && e3c / e2c < gamma_32) {
            eigenValues.push_back(e3c);
        }
    	else
    	{
    		eigenValues.push_back(0);
    	}

    	cov.Zero();
    	
    	//cout <<  (e1c) << ", " << e2c << ", " << e3c << std::endl;

    	indices.clear ();
    	std::vector<int> ().swap(indices);
    	
    	dist.clear();
    	std::vector<double> ().swap(dist);
    }

    std::vector<int> kp_indices = std::vector<int> ();
    //kp_indices.reserve(input.size());
//#pragma omp parallel for schedule(static) shared(kp_indices)
    for (int i = 0; i < (int)input.size(); i++) {
        if (eigenValues[i] > 0.0) {
            std::vector<int> nn_indices;
            std::vector<double> dist;
            int nb_neighbors = kdtree.SearchRadius(input[i], non_max_radius,
                                                   nn_indices, dist);

            if (nb_neighbors >= min_neighbors && IsLocalMaxima(i, nn_indices, eigenValues)) {
                kp_indices.push_back (i);
            }

        	nn_indices.clear ();
        	std::vector<int> ().swap(nn_indices);
    	
        	dist.clear();
        	std::vector<double> ().swap(dist);
        }
    }
    
	

    // utility::LogDebug("[ComputeISSKeypoints] Extracted {} keypoints",
    //                   kp_indices.size());
    return kp_indices;
}

EVertexType Descriptor_ISS::GetVertexType(int index, const int depth, const float dotFlat0, const float dotFlat1)
{
	EVertexType ret = EVertexType::NONE;

	std::queue<int> queue;
	std::unordered_map<int, bool> visited;
    
	// enqueue
	queue.push(index);
	queue.push(-1);
	visited[index] = true;
	int levels = 0;

	std::vector<int> neighborIndices = std::vector<int>();
    
	// 주변 이웃 인덱스 구하기
	while (!queue.empty())
	{
		int v = queue.front();
		queue.pop();
	    
		if (v == -1)
		{
			levels++;
			queue.push(-1);
			if (queue.front() == -1 || levels == depth) break;
		} 
		else
		{
			for (int u : meshData->neighbors[v])
			{
				if (!visited[u])
				{
					neighborIndices.push_back(u);

					visited[u] = true;
					queue.push (u);
				}
			}
		}
	}

	// 초기화
	while (!queue.empty())
		queue.pop();
	
	visited.clear();
	std::unordered_map<int, bool>().swap(visited);
	
	// 얻은 주변 인덱스를 이용하여 Type 계산
	if (neighborIndices.size() == 0)
		return EVertexType::NONE;

	FVector neighboursPos_sum = FVector::Zero();
	FVector neighboursPos_avg = FVector::Zero();

	for (int i = 0; i != neighborIndices.size(); i++)
		neighboursPos_sum += FVector(meshData->positions[neighborIndices[i]].x(), meshData->positions[neighborIndices[i]].y(), meshData->positions[neighborIndices[i]].z());

	neighboursPos_avg = neighboursPos_sum / (1.0 * neighborIndices.size());

	FVector direction_self = FVector(meshData->normals[index].x(), meshData->normals[index].y(), meshData->normals[index].z());
	FVector direction_avg = FVector(meshData->positions[index].x(), meshData->positions[index].y(), meshData->positions[index].z()) - neighboursPos_avg;
	direction_avg = direction_avg / FVector::Distance(FVector::Zero(), direction_avg);

	float dotP = FVector::DotProduct(direction_self, direction_avg);
	
	if (dotFlat0 < dotP && dotP <= dotFlat1)
	{
		ret = EVertexType::VERTEX_FLAT;
	}
	else
	{
		if (dotP > 0)
		{
			ret = EVertexType::VERTEX_BUMP;
		}
		else
		{
			ret = EVertexType::VERTEX_SINK;
		}
	}

	neighborIndices.clear();
	std::vector<int>().swap(neighborIndices);

	return ret;
}

EVertexNormalType Descriptor_ISS::GetVertexNormalType(int index, const float dotUp, const float dotDown)
{
	FVector nor = FVector(meshData->normals[index].x(), meshData->normals[index].y(), meshData->normals[index].z());
    
	// 노멀 구분하기  
	float dot = FVector::DotProduct(nor, FVector::UpVector);

	if (dot > dotUp)
	{
		return EVertexNormalType::VERTEX_UP;
	}
	
	if (dot < dotDown)
	{
		return EVertexNormalType::VERTEX_DOWN;
	}

	return EVertexNormalType::VERTEX_PARALLEL;
}

// ================================================================================================================================

void Descriptor_ISS::InitKeypoints(std::vector<int>& vrts_selected, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	meshData->Clear();
	eigenValues.clear ();
	vector<double>().swap(eigenValues);
	
	if (!MyUtil::ReadMeshWithoutOverwrap(m_pMeshCom, *meshData))
		return;
		
	// iss
	vrts_selected.clear();
	std::vector <int>().swap(vrts_selected);
	
	vrts_selected = ComputeISSKeypoints(meshData->positions, m_saliencyRaidus, m_maxRadius, m_gamma_21, m_gamma_32, m_minNeighbors);

	for (int ind : vrts_selected)
	{
		vrts_postSelected.Add(ind);
	}
}