#include "Descriptor_MeshSaliency.h"

void Descriptor_MeshSaliency::computeSaliency()
{
	 const int levels = 5;
    
    int count = (int)myMesh->vertices.size();
    double minSaliency = INFINITY;
    double maxSaliency = -INFINITY;
    std::vector<double> levelSaliencies(count);
    
    // 1: compute mean curvature
    computeMeanCurvature();
    
    // 2: initialize and compute extent
    BoundingBox bbox;
    for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
        bbox.expandToInclude(v->position);
    }
	// TODO 수정
	// 길이의 제곱
    double extent = 0.003 * 0.003 * bbox.extent.squaredNorm();
    
    // 3
	// {2, 3, 4, 5, 6}
    for (int i = 0; i < levels; i++) {
        
        // compute level saliencies
        double sumSaliency = 0.0;
        double distance2 = (i+2)*(i+2)*extent;
        for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
            
            double weightedCurvature1 = v->computeWeightedCurvature(v, distance2);
            double weightedCurvature2 = v->computeWeightedCurvature(v, 4*distance2);
            
            levelSaliencies[v->index] = std::abs(weightedCurvature1 - weightedCurvature2);
            sumSaliency += levelSaliencies[v->index];
        }
        
        // normalize
        double maxLevelSaliency = -INFINITY;
        for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
            levelSaliencies[v->index] /= sumSaliency;
            if (maxLevelSaliency < levelSaliencies[v->index]) maxLevelSaliency = levelSaliencies[v->index];
        }
        
        // compute mean of local maxima
        double peaks = 0.0;
        double meanLocalMaxSaliency = 0.0;
        for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
            if (levelSaliencies[v->index] != maxLevelSaliency && v->isPeakSaliency(v, levelSaliencies)) {
                meanLocalMaxSaliency += levelSaliencies[v->index];
                peaks += 1.0;
            }
        }
        meanLocalMaxSaliency /= peaks;
        
        // apply non-linear suppression operator to level saliency
        double suppressionFactor = pow(maxLevelSaliency - meanLocalMaxSaliency, 2);
        for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
            v->saliency += levelSaliencies[v->index] * suppressionFactor;
            
            if (i+1 == levels) {
                if (v->saliency < minSaliency) minSaliency = v->saliency;
                if (maxSaliency < v->saliency) maxSaliency = v->saliency;
            }
        }
    }
    
    // 4: scale between 0 and 1
    double dSaliency = maxSaliency - minSaliency;
    for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
        v->saliency = (v->saliency - minSaliency) / dSaliency;
    }
}

void Descriptor_MeshSaliency::buildLaplacian(Eigen::SparseMatrix<double>& L) const
{
	std::vector<Eigen::Triplet<double>> LTriplet;
    
	for (VertexCIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
        
		HalfEdgeCIter he = v->he;
		double dualArea = v->dualArea();
		double sumCoefficients = 0.0;
		do {
			// (cotA + cotB) / 2A
			double coefficient = 0.5 * (he->cotan() + he->flip->cotan()) / dualArea;
			sumCoefficients += coefficient;
            
			LTriplet.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, coefficient));
            
			he = he->flip->next;
		} while (he != v->he);
        
		LTriplet.push_back(Eigen::Triplet<double>(v->index, v->index, -sumCoefficients));
	}
    
	L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

void Descriptor_MeshSaliency::computeMeanCurvature()
{
	int _v = (int)myMesh->vertices.size();
	Eigen::SparseMatrix<double> L(_v, _v);
	buildLaplacian(L);
    
	Eigen::MatrixXd x;
	x.resize(_v, 3);
	for (VertexCIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		x.row(v->index) = v->position;
	}
	x = L * x;
    
	for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		v->meanCurvature = 0.5 * x.row(v->index).norm();
	}
}

void Descriptor_MeshSaliency::normalize()
{
	// compute center of mass
	Eigen::Vector3d cm = Eigen::Vector3d::Zero();
	for (VertexCIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		cm += v->position;
	}
	cm /= (double)myMesh->vertices.size();
    
	// translate to origin
	for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		v->position -= cm;
	}
    
	// determine radius
	double rMax = 0;
	for (VertexCIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		rMax = std::max(rMax, v->position.norm());
	}
    
	// rescale to unit sphere
	for (VertexIter v = myMesh->vertices.begin(); v != myMesh->vertices.end(); v++) {
		v->position /= rMax;
	}
}

void Descriptor_MeshSaliency::InitKeypoints(std::vector<int>& vrts_selected, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	if (myMesh->GetIsEnableModel() == false)
		return;

	if (myMesh->GetIsEnableModel() == false)
		return;

	computeSaliency ();

	for (EdgeCIter e = myMesh->edges.begin(); e != myMesh->edges.end(); e ++)
	{
		VertexIter a = e->he->vertex;
		VertexIter b = e->he->flip->vertex;

		if (a->saliency > m_cutoffSaliency && a->isPeakSaliency(a)) {
			// glVertex3d(a->position.x(), a->position.y(), a->position.z());
			vrts_selected.push_back(a->index);
		}
            
		if (b->saliency > m_cutoffSaliency && b->isPeakSaliency(b)) {
			// glVertex3d(b->position.x(), b->position.y(), b->position.z());
			vrts_selected.push_back(b->index);
		}
	}
	
	for (int vrts : vrts_selected)
	{
		if (vrts_postSelected.Contains(vrts))
			continue;
		
		vrts_postSelected.Add(vrts);
	}
}
