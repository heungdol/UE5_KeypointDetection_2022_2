// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
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

	// computes mesh saliency
	void computeSaliency();
	
	void buildLaplacian(Eigen::SparseMatrix<double>& L) const;
    
	// computes mean curvature per vertex
	void computeMeanCurvature();
        
	// center mesh about origin and rescale to unit radius
	void normalize();

public:

	virtual void InitKeypointDetection () override;
	virtual void InitSelectedVertexLocation () override;
	
	Mesh myMesh;
	
	void CalculateMeshSaliency ();

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Mesh Saliency")
	double m_cutoffSaliency = 0.75;
};
