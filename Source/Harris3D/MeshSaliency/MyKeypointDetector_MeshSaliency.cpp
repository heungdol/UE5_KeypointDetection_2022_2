// Fill out your copyright notice in the Description page of Project Settings.


#include "MyKeypointDetector_MeshSaliency.h"

void AMyKeypointDetector_MeshSaliency::OnConstruction(const FTransform& Transform)
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

		myMesh.Clear ();
		myMesh = Mesh (m_pMeshCom);

		// 만약 적절하지 않은 모델이라면
		// (버텍스 개수가 50000개 이상인 경우)
		if (myMesh.GetIsEnableModel() == false)
			return;	

		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		vector <int>().swap(vrts_selected);
		
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

void AMyKeypointDetector_MeshSaliency::BeginPlay()
{
	Super::BeginPlay();

	UpdateSelectedVertexLocation();
}

void AMyKeypointDetector_MeshSaliency::InitKeypointDetection()
{
	Super::InitKeypointDetection();

	CalculateMeshSaliency();
}

void AMyKeypointDetector_MeshSaliency::InitSelectedVertexLocation()
{
	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		vrtLocs_postSelected.Push(myMesh.GetVertexLocByIndex(vrts_postSelected[i]));
		currentVrtLocs_postSelected.Push(myMesh.GetVertexLocByIndex(vrts_postSelected[i]));
		
		vrtNors_postSelected.Push (myMesh.GetVertexNorByIndex (vrts_postSelected[i]));
		currentVrtNors_postSelected.Push (myMesh.GetVertexNorByIndex (vrts_postSelected[i]));

		EVertexType vertexType = myMesh.vertices[vrts_postSelected[i]].getVertexType(myMesh.meshData, _dotFlat0, _dotFlat1, m_vertexType_depth);
		EVertexNormalType vertexNormalType = myMesh.vertices[vrts_postSelected[i]].getVertexNormalType(myMesh.meshData, _dotUp, _dotDown);
		vrtTypes_postSelected.Push(vertexType);
		vrtNorTypes_postSelected.Push(vertexNormalType);
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
	
	PrintDetectionInfo ();
}

void AMyKeypointDetector_MeshSaliency::CalculateMeshSaliency()
{
	if (myMesh.GetIsEnableModel() == false)
		return;

	computeSaliency ();

	for (EdgeCIter e = myMesh.edges.begin(); e != myMesh.edges.end(); e ++)
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

void AMyKeypointDetector_MeshSaliency::computeSaliency()
{
	 const int levels = 5;
    
    int count = (int)myMesh.vertices.size();
    double minSaliency = INFINITY;
    double maxSaliency = -INFINITY;
    std::vector<double> levelSaliencies(count);
    
    // 1: compute mean curvature
    computeMeanCurvature();
    
    // 2: initialize and compute extent
    BoundingBox bbox;
    for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
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
        for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
            
            double weightedCurvature1 = v->computeWeightedCurvature(v, distance2);
            double weightedCurvature2 = v->computeWeightedCurvature(v, 4*distance2);
            
            levelSaliencies[v->index] = std::abs(weightedCurvature1 - weightedCurvature2);
            sumSaliency += levelSaliencies[v->index];
        }
        
        // normalize
        double maxLevelSaliency = -INFINITY;
        for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
            levelSaliencies[v->index] /= sumSaliency;
            if (maxLevelSaliency < levelSaliencies[v->index]) maxLevelSaliency = levelSaliencies[v->index];
        }
        
        // compute mean of local maxima
        double peaks = 0.0;
        double meanLocalMaxSaliency = 0.0;
        for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
            if (levelSaliencies[v->index] != maxLevelSaliency && v->isPeakSaliency(v, levelSaliencies)) {
                meanLocalMaxSaliency += levelSaliencies[v->index];
                peaks += 1.0;
            }
        }
        meanLocalMaxSaliency /= peaks;
        
        // apply non-linear suppression operator to level saliency
        double suppressionFactor = pow(maxLevelSaliency - meanLocalMaxSaliency, 2);
        for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
            v->saliency += levelSaliencies[v->index] * suppressionFactor;
            
            if (i+1 == levels) {
                if (v->saliency < minSaliency) minSaliency = v->saliency;
                if (maxSaliency < v->saliency) maxSaliency = v->saliency;
            }
        }
    }
    
    // 4: scale between 0 and 1
    double dSaliency = maxSaliency - minSaliency;
    for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
        v->saliency = (v->saliency - minSaliency) / dSaliency;
    }
}

void AMyKeypointDetector_MeshSaliency::buildLaplacian(Eigen::SparseMatrix<double>& L) const
{
	std::vector<Eigen::Triplet<double>> LTriplet;
    
	for (VertexCIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
        
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

void AMyKeypointDetector_MeshSaliency::computeMeanCurvature()
{
	int _v = (int)myMesh.vertices.size();
	Eigen::SparseMatrix<double> L(_v, _v);
	buildLaplacian(L);
    
	Eigen::MatrixXd x;
	x.resize(_v, 3);
	for (VertexCIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		x.row(v->index) = v->position;
	}
	x = L * x;
    
	for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		v->meanCurvature = 0.5 * x.row(v->index).norm();
	}
}

void AMyKeypointDetector_MeshSaliency::normalize()
{
	// compute center of mass
	Eigen::Vector3d cm = Eigen::Vector3d::Zero();
	for (VertexCIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		cm += v->position;
	}
	cm /= (double)myMesh.vertices.size();
    
	// translate to origin
	for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		v->position -= cm;
	}
    
	// determine radius
	double rMax = 0;
	for (VertexCIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		rMax = std::max(rMax, v->position.norm());
	}
    
	// rescale to unit sphere
	for (VertexIter v = myMesh.vertices.begin(); v != myMesh.vertices.end(); v++) {
		v->position /= rMax;
	}
}
