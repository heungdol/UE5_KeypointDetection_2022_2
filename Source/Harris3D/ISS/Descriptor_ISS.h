#pragma once

#include <ThirdParty/Eigen/Eigen/Eigen>
#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Eigenvalues>
#include <ThirdParty/Eigen/Eigen/Geometry>
#include <ThirdParty/Eigen/Eigen/Sparse>
#include "Harris3D/MyUtil/Mesh.h"
#include "KDTreeFlann.h"

using namespace std;

class Descriptor_ISS
{
public:

	Descriptor_ISS () {}
	Descriptor_ISS (UStaticMeshComponent* sm, MeshData* md
		, double saliencyRadius, double maxRadius, double gamma21, double gamma32, int minNeighbors)
			: m_pMeshCom(sm), meshData(md), m_saliencyRaidus(saliencyRadius), m_maxRadius(maxRadius)
	, m_gamma_21(gamma21), m_gamma_32(gamma32), m_minNeighbors(minNeighbors)
	{}
	
	double m_saliencyRaidus = 1;
	double m_maxRadius = 10;
	double m_gamma_21 = 0.975f;
	double m_gamma_32 = 0.975f;
	int m_minNeighbors = 5;
	
	virtual string GetDetectorName()
	{
		return "========== Intrinsic Shape Signature ==========";
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
		cout << "m_saliencyRaidus: " << m_saliencyRaidus << std::endl;
		cout << "m_maxRadius: " << m_maxRadius << std::endl;
		cout << "m_gamma_21: " << m_gamma_21 << std::endl;
		cout << "m_gamma_32: " << m_gamma_32 << std::endl;
		cout << "m_minNeighbors: " << m_minNeighbors << std::endl;

		
		// cout << std::endl;
		// cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		// cout << "Area: " << meshData.GetArea() << "(m^2)"<< std::endl;

		cout << std::endl;
		cout << "Total Keypoint Number: " << meshData->positions.size() << std::endl;
		//
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
	void InitKeypoints (std::vector<int>&, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);

	EVertexType GetVertexType (int index, const int depth, const float dotFlat0, const float dotFlat1);
	EVertexNormalType GetVertexNormalType (int index, const float dotUp, const float dotDown);

	std::vector<double> eigenValues = std::vector<double>();
	std::vector<int> ComputeISSKeypoints(const std::vector<Eigen::Vector3d> &input,
												double salient_radius = 0.0,
												double non_max_radius = 0.0,
												double gamma_21 = 0.975,
												double gamma_32 = 0.975,
												int min_neighbors = 5);

	// 메쉬 데이터
	UStaticMeshComponent* m_pMeshCom;
	MeshData* meshData;
	KDTreeFlann kdtree;
	//Eigen::MatrixXd _input;
};
