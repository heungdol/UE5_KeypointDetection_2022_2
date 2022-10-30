// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
//#pragma warning(disable: 4668)



#include "../MyKeypointDetector.h"


#include <ThirdParty/Eigen/Eigen/Eigen>
#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Eigenvalues>
#include <ThirdParty/Eigen/Eigen/Geometry>
#include <ThirdParty/Eigen/Eigen/Sparse>
#include "KDTreeFlann.h"

#include "../MyUtil/MyUtil.h"


#include "CoreMinimal.h"
#include "Harris3D/MyUtil/Mesh.h"
#include "MyKeypointDetector_ISS.generated.h"

/**
 * 
 */
UCLASS()
class HARRIS3D_API AMyKeypointDetector_ISS : public AMyKeypointDetector
{
	GENERATED_BODY()
	
public:
	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	virtual void BeginPlay() override;

public:

	virtual void InitKeypointDetection () override;

	virtual void InitSelectedVertexLocation () override;
	//virtual void UpdateSelectedVertexLocation () override;

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

		
		cout << std::endl;
		cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		cout << "Area: " << meshData.GetArea() << "(m^2)"<< std::endl;

		cout << std::endl;
		cout << "Total Keypoint Number: " << vrts_postSelected.Num() << std::endl;

		int countUp = 0;
		int countParallel = 0;
		int countDown = 0;

		int countBump = 0;
		int countFlat = 0;
		int countSink = 0;

		// 노멀 구분하기
		for (EVertexType type : vrtTypes_postSelected)
		{
			switch (type)
			{
			default:
				break;
			case EVertexType::VERTEX_BUMP:
				countBump++;
				break;
			case EVertexType::VERTEX_FLAT:
				countFlat++;
				break;
			case EVertexType::VERTEX_SINK:
				countSink++;
				break;
			}
		}

		for (EVertexNormalType type : vrtNorTypes_postSelected)
		{
			switch (type)
			{
			default:
				break;
			case EVertexNormalType::VERTEX_UP:
				countUp++;
				break;
			case EVertexNormalType::VERTEX_PARALLEL:
				countParallel++;
				break;
			case EVertexNormalType::VERTEX_DOWN:
				countDown++;
				break;
			}
		}
		
		cout << "Normal Number (Bump): " << countBump << std::endl;
		cout << "Normal Number (Flat): " << countFlat << std::endl;
		cout << "Normal Number (Sink): " << countSink << std::endl;

		cout << "Normal Number (Up): " << countUp << std::endl;
		cout << "Normal Number (Parellel): " << countParallel << std::endl;
		cout << "Normal Number (Down): " << countDown << std::endl;
	}

	// 메쉬 데이터
	MeshData meshData;
	KDTreeFlann kdtree;
	//Eigen::MatrixXd _input;

	// UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	// double m_model_resolution = 1;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_saliencyRaidus = 1;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_maxRadius = 10;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_gamma_21 = 0.975f;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_gamma_32 = 0.975f;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	int m_minNeighbors = 5;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	int m_vertexType_depth = 5;

	// UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	// int m_numberOfThreads = 4;
	
	std::vector<double> eigenValues = std::vector<double>();
	std::vector<int> ComputeISSKeypoints(const std::vector<Eigen::Vector3d> &input,
												double salient_radius = 0.0,
												double non_max_radius = 0.0,
												double gamma_21 = 0.975,
												double gamma_32 = 0.975,
												int min_neighbors = 5);

	EVertexType GetVertexType (int index, const int depth, const float dotFlat0, const float dotFlat1);
	EVertexNormalType GetVertexNormalType (int index, const float dotUp, const float dotDown);
};

