// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "MyMesh.h"

#include "CoreMinimal.h"
#include "Descriptor_Harris3D.h"
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
	
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	int m_ringSize = 5;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	double m_fraction = 0.01;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	double m_k = 0.04;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	int m_vertexType_depth = 5;

	MyMesh myMesh;
	Descriptor_Harris3D myDescriptor;

	virtual void InitKeypointDetection () override;
	virtual void InitSelectedVertexLocation () override;
	
	//virtual void UpdateSelectedVertexLocation () override;
};
