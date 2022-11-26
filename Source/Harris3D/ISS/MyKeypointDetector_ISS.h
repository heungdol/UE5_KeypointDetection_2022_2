// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
//#pragma warning(disable: 4668)



#include "../MyKeypointDetector.h"


#include <ThirdParty/Eigen/Eigen/Eigen>
#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Eigenvalues>
#include <ThirdParty/Eigen/Eigen/Geometry>
#include <ThirdParty/Eigen/Eigen/Sparse>
//#include "KDTreeFlann.h"

#include "../MyUtil/MyUtil.h"


#include "CoreMinimal.h"
#include "Descriptor_ISS.h"
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

	MeshData meshData;
	Descriptor_ISS myDescriptor;// = Descriptor_ISS(m_pMeshCom, meshData, m_saliencyRaidus, m_maxRadius, m_gamma_21, m_gamma_32, m_minNeighbors);

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
	
	
	};

