// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Descriptor_MeshSaliency.h"
#include "../MyKeypointDetector.h"
#include "../MyUtil/Mesh.h"
#include "MyKeypointDetector_MeshSaliency.generated.h"

/**
 * 
 */
UCLASS()
class HARRIS3D_API AMyKeypointDetector_MeshSaliency : public AMyKeypointDetector
{
	GENERATED_BODY()

public:
	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	virtual void BeginPlay() override;


public:

	virtual void InitKeypointDetection () override;
	virtual void InitSelectedVertexLocation () override;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Mesh Saliency")
	double m_cutoffSaliency = 0.75;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Mesh Saliency")
	int m_vertexType_depth = 5;

	Mesh myMesh;
	Descriptor_MeshSaliency descriptor;
};
