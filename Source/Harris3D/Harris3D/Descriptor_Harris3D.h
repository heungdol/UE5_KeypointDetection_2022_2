#pragma once

#include "MyMesh.h"

using namespace std;

class Descriptor_Harris3D
{
public:
	Descriptor_Harris3D (): myMesh(nullptr) {} ;
	Descriptor_Harris3D (MyMesh* mm, int ringsize, double frac, double k)
		: myMesh(mm), m_ringSize(ringsize), m_fraction(frac), m_k(k) {}
	~Descriptor_Harris3D() {}
	
	virtual string GetDetectorName()
	{
		return "========== Harris 3D ==========";
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
		cout << "m_ringSize: " << m_ringSize << std::endl;
		cout << "m_k: " << m_k << std::endl;
		//
		// cout << std::endl;
		// cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		// cout << "Area: " << myMesh.meshData.GetArea() << "(m^2)"<< std::endl;
		//
		cout << std::endl;
		cout << "Total Keypoint Number: " << myMesh->vertices.size() << std::endl;

		// int countUp = 0;
		// int countParallel = 0;
		// int countDown = 0;
		//
		// int countBump = 0;
		// int countFlat = 0;
		// int countSink = 0;
		//
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
	}
	
    int m_ringSize = 5;
    double m_fraction = 0.01;
    double m_k = 0.04;
    int m_vertexType_depth = 5;

	vector<double> harrisRPoints;
	MyMesh* myMesh;

	void InitKeypoints (std::vector<int>&, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);
	bool GetIsLocalMaxima(unsigned int) const;
	
};
