// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Dense>
#include <ThirdParty/Eigen/Eigen/SparseCore>

#include "../MyKeypointDetector.h"
#include "../MyUtil/Mesh.h"
#include "../MyUtil/MeshIO.h"
#include "Descriptor_HKS.h"

#include "CoreMinimal.h"
#include "MyKeypointDetector_HKS.generated.h"

UCLASS()
class HARRIS3D_API AMyKeypointDetector_HKS : public AMyKeypointDetector
{
	GENERATED_BODY()

public:
	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	virtual void BeginPlay() override;

public:

	virtual void InitKeypointDetection () override;
	virtual void InitSelectedVertexLocation () override;

	Mesh myMesh;
	Descriptor_HKS myDescriptor;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	int m_t = 5;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	int m_depth = 5;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	int m_vertexType_depth = 5;
};
