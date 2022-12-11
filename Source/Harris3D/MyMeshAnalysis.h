// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyUtil/KeypointDetectionBundle.h"
#include "MyUtil/MeshIO.h"
#include "MyUtil/VertexType.h"

#include "MyMeshAnalysis.generated.h"

using namespace std;

UCLASS()
class HARRIS3D_API AMyMeshAnalysis : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMyMeshAnalysis();

	void InitSelectedVertexLocation();
	void UpdateSelectedVertexLocation();
	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	UPROPERTY (VisibleAnywhere)	
	UStaticMeshComponent* m_pMeshCom;
	MeshData meshData;
	KeypointDetectionBundle keypointDetectionBundle;

	FVector actorLocation;
	FVector actorScale;
	FRotator actorRotation;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	bool m_update_click = false;
	bool m_update_first = false;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	int m_vertexNumber = 0;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	int m_vertexNumber_valid = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_surfaceArea = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_surfaceArea_valid = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_vertexRatioByArea = 0;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_vertexRatioByArea_valid = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_meshHeight = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	FVector m_boundingBoxSize = FVector::Zero();
	FVector m_boundingBoxCoord_min = FVector::Zero();
	FVector m_boundingBoxCoord_max = FVector::Zero();
	FVector m_boundingBoxCoord_pivot = FVector::Zero();;

	// ==========================================================================

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Voxel")
	bool m_voxelOn = false;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Voxel")
	double m_voxelGridSize = 1;
	int voxelGridNum_x = 0;
	int voxelGridNum_y = 0;
	int voxelGridNum_z = 0;
	std::vector <int> keypointNumberInVoxel = std::vector<int>{};
	std::vector <bool> keypointExistInVoxel = std::vector<bool>{};
	std::vector <FVector> voxelGridPivots = std::vector<FVector> {};

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Voxel")
	int m_voxelAbleThreshold = 4;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Voxel")
	FColor m_voxelAbleColor = FColorList::Green;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Voxel")
	FColor m_voxelFalseColor = FColorList::Red;

	// ==========================================================================

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Path")
	int m_totalPath = 0;

	UPROPERTY(EditAnywhere, Category="Mesh Analysis: Path")
	int m_drawPath = -1;

	std::vector<int> startPoints = std::vector <int>{};
	std::vector<std::vector<int>> path = std::vector <std::vector <int>>{};
	
	// ==========================================================================

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_debugDraw = false;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	EDetectorType m_detectorType = EDetectorType::NONE;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	FColor m_debugColor = FColorList::Orange;

		// ============================ Harris 3D =====================================

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	int m_ringSize = 5;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	double m_fraction = 0.01;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	double m_k = 0.04;

	// UPROPERTY(EditAnywhere, Category="Keypoint Detector: Harris 3D")
	// int m_vertexType_depth = 5;

	// ============================ HKS =====================================

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	int m_t = 5;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	int m_depth = 5;

	// UPROPERTY(EditAnywhere, Category="Keypoint Detector: Heat Kernel Signature")
	// int m_vertexType_depth = 5;

	// ============================ Mesh Saliency =====================================

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Mesh Saliency")
	double m_cutoffSaliency = 0.75;

	// ============================ ISS =====================================

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
	
	vector <int> vrts_selected;
	TArray <int> vrts_postSelected;
	TArray <FVector> vrtLocs_postSelected;
	TArray <FVector> vrtNors_postSelected;
	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <FVector> currentVrtLocs_postSelected;
	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <FVector> currentVrtNors_postSelected;
	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <EVertexType> vrtTypes_postSelected;
	UPROPERTY(BlueprintReadOnly, Category="Inspector")
	TArray <EVertexNormalType> vrtNorTypes_postSelected;
};
