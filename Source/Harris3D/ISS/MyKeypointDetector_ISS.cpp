#include "MyKeypointDetector_ISS.h"
// Fill out your copyright notice in the Description page of Project Settings.

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
	return resolution;
}

template <typename IdxType>
Eigen::Matrix3d ComputeCovariance(const std::vector<Eigen::Vector3d> &points,
								  const std::vector<IdxType> &indices)
{
	if (indices.empty()) {
		return Eigen::Matrix3d::Identity();
	}
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

std::vector<int> AMyKeypointDetector_ISS::ComputeISSKeypoints(
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

    	//cout <<  (e1c) << ", " << e2c << ", " << e3c << std::endl;
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
        }
    }

    // utility::LogDebug("[ComputeISSKeypoints] Extracted {} keypoints",
    //                   kp_indices.size());
    return kp_indices;
}

EVertexType AMyKeypointDetector_ISS::GetVertexType(int index, const int depth, const float dotFlat0, const float dotFlat1)
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
			for (int u : meshData.neighbors[v].neighborIndices)
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
	
	// 얻은 주변 인덱스를 이용하여 Type 계산
	if (neighborIndices.size() == 0)
		return EVertexType::NONE;

	FVector neighboursPos_sum = FVector::Zero();
	FVector neighboursPos_avg = FVector::Zero();

	for (int i = 0; i != neighborIndices.size(); i++)
		neighboursPos_sum += FVector(meshData.positions[neighborIndices[i]].x(), meshData.positions[neighborIndices[i]].y(), meshData.positions[neighborIndices[i]].z());

	neighboursPos_avg = neighboursPos_sum / (1.0 * neighborIndices.size());

	FVector direction_self = FVector(meshData.normals[index].x(), meshData.normals[index].y(), meshData.normals[index].z());
	FVector direction_avg = FVector(meshData.positions[index].x(), meshData.positions[index].y(), meshData.positions[index].z()) - neighboursPos_avg;
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

	return ret;
}

EVertexNormalType AMyKeypointDetector_ISS::GetVertexNormalType(int index, const float dotUp, const float dotDown)
{
	FVector nor = FVector(meshData.normals[index].x(), meshData.normals[index].y(), meshData.normals[index].z());
    
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

void AMyKeypointDetector_ISS::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 첫 실행 혹은 에디터에서 갱신 bool를 활성화할 시
	if (m_update_first == false || m_update_click == true)
	{
		m_update_first = true;
		m_update_click = false;
		
		// 메쉬 판단
		if (!m_pMeshCom)
			return;
		
		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		vrts_postSelected.Empty();
		
		vrtLocs_postSelected.Empty();
		vrtNors_postSelected.Empty();

		vrtTypes_postSelected.Empty();
		vrtNorTypes_postSelected.Empty();
		
		currentVrtLocs_postSelected.Empty();
		currentVrtNors_postSelected.Empty();
		
		InitKeypointDetection ();

		InitSelectedVertexLocation ();
		UpdateSelectedVertexLocation();
	}
}

void AMyKeypointDetector_ISS::BeginPlay()
{
	Super::BeginPlay();
}

void AMyKeypointDetector_ISS::InitKeypointDetection()
{
	Super::InitKeypointDetection();

	meshData.indices.clear();
	meshData.normals.clear();
	meshData.positions.clear();
	meshData.uvs.clear();

	if (!MyUtil::ReadMeshWithoutOverwrap(m_pMeshCom, meshData))
		return;
		
	// iss
	vrts_selected = ComputeISSKeypoints(meshData.positions, m_saliencyRaidus, m_maxRadius, m_gamma_21, m_gamma_32, m_minNeighbors);

	for (int ind : vrts_selected)
	{
		vrts_postSelected.Add(ind);
	}
}

void AMyKeypointDetector_ISS::InitSelectedVertexLocation()
{
	Super::InitSelectedVertexLocation();

	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		FVector pos;
		pos.X = meshData.positions[vrts_postSelected[i]][0];
		pos.Y = meshData.positions[vrts_postSelected[i]][1];
		pos.Z = meshData.positions[vrts_postSelected[i]][2];

		FVector no;
		no.X = meshData.normals[vrts_postSelected[i]][0];
		no.Y = meshData.normals[vrts_postSelected[i]][1];
		no.Z = meshData.normals[vrts_postSelected[i]][2];
		
		vrtLocs_postSelected.Add(pos);
		currentVrtLocs_postSelected.Add(pos);
		
		vrtNors_postSelected.Add(no);
		currentVrtNors_postSelected.Add(no);

		vrtTypes_postSelected.Push(GetVertexType(vrts_postSelected[i], m_vertexType_depth, _dotFlat0, _dotFlat1));
		vrtNorTypes_postSelected.Push(GetVertexNormalType(vrts_postSelected[i], _dotUp, _dotDown));
	}

	for (int i = 0; i < vrtLocs_postSelected.Num(); i++)
	{
		//GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Yellow, TEXT("OHa"));

		FVector offset = vrtLocs_postSelected[i];
		offset *= actorScale;
		offset = actorRotation.RotateVector(offset);
		
		currentVrtLocs_postSelected [i] = actorLocation + offset;
		currentVrtNors_postSelected [i] = actorRotation.RotateVector(vrtNors_postSelected [i]);
	}

	PrintDetectionInfo();
}

/*
void AMyKeypointDetector_ISS::UpdateSelectedVertexLocation()
{
	Super::UpdateSelectedVertexLocation();

	FTimerHandle debugDrawWaitHandle;
	
	float WaitTime = 0.1; //시간 설정
	//bool  firstInit =  false;

	if (GetWorld())
		GetWorld()->GetTimerManager().SetTimer(debugDrawWaitHandle, FTimerDelegate::CreateLambda([&]()
		{
				for (int i = 0; i < vrtLocs_postSelected.Num(); i++)
				{
					//GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Yellow, TEXT("OHa"));

					FVector offset = vrtLocs_postSelected[i];
					offset *= actorScale;
					offset = actorRotation.RotateVector(offset);
				
					currentVrtLocs_postSelected [i] = actorLocation + offset;
					currentVrtNors_postSelected [i] = actorRotation.RotateVector(vrtNors_postSelected [i]);

					if (m_debugDraw == true)
					{
						DrawDebugLine(GetWorld()
							, currentVrtLocs_postSelected[i], currentVrtLocs_postSelected[i]+4*currentVrtNors_postSelected[i]
							, FColorList::Red, false, 0.1, 0, 1);
					}
				}

				actorLocation = GetActorLocation();
				actorScale = GetActorScale();
				actorRotation = GetActorRotation();

			GetWorld()->GetTimerManager().ClearTimer(debugDrawWaitHandle);
		}), WaitTime, true); //반복도 여기서 추가 변수를 선언해 설정가능
}
*/
