// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "MyMesh.h"

#include "CoreMinimal.h"
#include "../MyKeypointDetector.h"
#include "MyKeypointDetector_Harris.generated.h"

/**
 * 
 */
UCLASS()
class HARRIS3D_API AMyKeypointDetector_Harris : public AMyKeypointDetector
{
	GENERATED_BODY()

public:
	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

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
		
		cout << std::endl;
		cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		cout << "Area: " << myMesh.meshData.GetArea() << "(m^2)"<< std::endl;
		
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
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	int m_ringSize = 5;
	
	// UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	// double m_fraction = 0.01;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	double m_k = 0.04;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	int m_vertexType_depth = 5;

	MyMesh myMesh;
	
	int ringSize;
	double fraction_constant;
	double k_parameter;
	vector<double> harrisRPoints;

	virtual void InitKeypointDetection () override;
	virtual void InitSelectedVertexLocation () override;

	void CalculateHarrisResponse ();
	bool GetIsLocalMaxima(unsigned int);
	
	//virtual void UpdateSelectedVertexLocation () override;
};
