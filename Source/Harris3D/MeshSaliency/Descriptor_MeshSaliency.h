#pragma once

#include "../MyUtil/Mesh.h"

using namespace std;

class Descriptor_MeshSaliency
{
public:
	Descriptor_MeshSaliency (): myMesh(nullptr){}
	Descriptor_MeshSaliency (Mesh* mesh, float cutoff) : myMesh(mesh), m_cutoffSaliency(cutoff) {};
	~Descriptor_MeshSaliency() {}

	virtual string GetDetectorName()
	{
		return "========== Mesh Saliency ==========";
	};

	virtual void PrintDetectionInfo ()
	{
		// 출력
		// 이름
		// 파라미터
		// 전체 Keypoint 개수
		// Normal 별 개수
		// Type 별 개수

		cout << std::endl;
		cout << GetDetectorName () << std::endl;
		cout << std::endl;
		cout << "m_cutoffSaliency: " << m_cutoffSaliency << std::endl;

		// cout << std::endl;
		// cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		// cout << "Area: " << myMesh.meshData.GetArea() << "(m^2)"<< std::endl;
		
		cout << std::endl;
		cout << "Total Keypoint Number: " << myMesh->meshData.positions.size() << std::endl;

		// int countUp = 0;
		// int countParallel = 0;
		// int countDown = 0;
		//
		// int countBump = 0;
		// int countFlat = 0;
		// int countSink = 0;

		// // 노멀 구분하기
		// for (EVertexType type : vrtTypes_postSelected)
		// {
		// 	switch (type)
		// 	{
		// 	default:
		// 		break;
		// 	case EVertexType::VERTEX_BUMP:
		// 		countBump++;
		// 		break;
		// 	case EVertexType::VERTEX_FLAT:
		// 		countFlat++;
		// 		break;
		// 	case EVertexType::VERTEX_SINK:
		// 		countSink++;
		// 		break;
		// 	}
		// }
		//
		// for (EVertexNormalType type : vrtNorTypes_postSelected)
		// {
		// 	switch (type)
		// 	{
		// 	default:
		// 		break;
		// 	case EVertexNormalType::VERTEX_UP:
		// 		countUp++;
		// 		break;
		// 	case EVertexNormalType::VERTEX_PARALLEL:
		// 		countParallel++;
		// 		break;
		// 	case EVertexNormalType::VERTEX_DOWN:
		// 		countDown++;
		// 		break;
		// 	}
		// }
		//
		// cout << "Normal Number (Bump): " << countBump << std::endl;
		// cout << "Normal Number (Flat): " << countFlat << std::endl;
		// cout << "Normal Number (Sink): " << countSink << std::endl;
		//
		// cout << "Normal Number (Up): " << countUp << std::endl;
		// cout << "Normal Number (Parellel): " << countParallel << std::endl;
		// cout << "Normal Number (Down): " << countDown << std::endl;
		//
		// cout << "======================================================" << endl;
		// for (Vertex v : myMesh.vertices)
		// {
		// 	cout << v.meanCurvature << endl;
		// }
		
	}

	// computes mesh saliency
	void computeSaliency();
	
	void buildLaplacian(Eigen::SparseMatrix<double>& L) const;
    
	// computes mean curvature per vertex
	void computeMeanCurvature();
        
	// center mesh about origin and rescale to unit radius
	void normalize();

	void CalculateMeshSaliency ();

	void InitKeypoints (std::vector<int>&, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);

	double m_cutoffSaliency = 0.75;
	// int m_vertexType_depth = 5;

	Mesh* myMesh;
};
