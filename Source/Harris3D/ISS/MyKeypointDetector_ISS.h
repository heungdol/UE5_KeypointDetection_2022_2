// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
//#pragma warning(disable: 4668)



#include "../MyKeypointDetector.h"


// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
//
// #include <pcl/search/kdtree.h>

#include <pcl/keypoints/iss_3d.h>

#include "CoreMinimal.h"
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
	virtual void UpdateSelectedVertexLocation () override;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_model_resolution = 1;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_saliencyRaidus = 1;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_gamma_21 = 0.975f;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	double m_gamma_32 = 0.975f;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	int m_minNeighbors = 5;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Intrinsic Shape Signature")
	int m_numberOfThreads = 4;
	
};
