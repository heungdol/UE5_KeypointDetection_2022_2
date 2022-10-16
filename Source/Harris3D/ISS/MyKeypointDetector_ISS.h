// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#pragma warning(disable: 4668)

#include "../MyKeypointDetector.h"

THIRD_PARTY_INCLUDES_START

#include <boost/thread/mutex.hpp>

THIRD_PARTY_INCLUDES_END

//#include <pcl/keypoints/keypoint.h>

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
	
};
