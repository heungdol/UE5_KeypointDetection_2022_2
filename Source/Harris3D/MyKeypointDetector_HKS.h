// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Dense>
#include <ThirdParty/Eigen/Eigen/SparseCore>

#include "MyMesh_HKS.h"

#include "CoreMinimal.h"
#include "MyKeypointDetector.h"
#include "MyKeypointDetector_HKS.generated.h"

/**
 * 
 */
UCLASS()
class HARRIS3D_API AMyKeypointDetector_HKS : public AMyKeypointDetector
{
	GENERATED_BODY()

public:
	virtual void OnConstruction(const FTransform& Transform) override;

private:
	MyMesh_HKS *mesh;
	Eigen::VectorXd evals;
	Eigen::MatrixXd evecs;

protected:
	virtual void BeginPlay() override;

public:
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: HKS")
	int m_t = 1;
	

	virtual void InitKeypointDetection () override;

	virtual void InitSelectedVertexLocation () override;
	virtual void UpdateSelectedVertexLocation () override;

	void InitHKS ();
	void ComputeHKS();
};
