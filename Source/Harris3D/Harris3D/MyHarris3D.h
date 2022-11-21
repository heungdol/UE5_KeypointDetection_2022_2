// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include <set>

#include "MeshDescription.h"
#include "MyMesh.h"

#include "../Harris3D/MyKeypointDetector_Harris.h"
#include "../HKS/MyKeypointDetector_HKS.h"
#include "../ISS/MyKeypointDetector_ISS.h"
#include "../MeshSaliency/MyKeypointDetector_MeshSaliency.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyHarris3D.generated.h"

using namespace std;

// ================================================================================================

UENUM(BlueprintType)
enum class EDetectorType : uint8
{
	DT_HARRIS3D
	, DT_HKS
	, DT_ISS 
	, DT_MESHSALIENCY 
};


UCLASS()
class HARRIS3D_API AMyHarris3D : public AActor
{
	GENERATED_BODY()
	
public:
	virtual void OnConstruction(const FTransform& Transform) override;

	// Sets default values for this actor's properties
	AMyHarris3D();
	
	/*
	UFUNCTION(BlueprintCallable)
	void UpdateHarris3D ();

	UFUNCTION(BlueprintCallable)
	bool GetIsUpdated ();
	*/
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	bool m_update_first = false;

public:
	// ============================== default =======================================
	
	UPROPERTY (VisibleAnywhere)
	UStaticMeshComponent* m_pMeshCom;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_debugDraw = false;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_update_click = false;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	EDetectorType m_detectorType = EDetectorType::DT_HARRIS3D;
	
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

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	int typeSelection;
	double fraction_constant;
	double k_parameter;
	
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
	
	// 논문에 제시된 클러스터링은 사용하지 않음
	//std::vector<int> selectedVrts_clustering;
	//TArray <FVector> selectedVrtLocs_clustering;
	//TArray <FVector> selectedVrtNors_clustering;
	//TArray <FVector> currentSelectedVrtLocs_clustering;
	//TArray <FVector> currentSelectedVrtNors_clustering;
	
	FVector actorLocation;
	FVector actorScale;
	FRotator actorRotation;
	
	FVector GetVertexLocationByIndex (int i);
	FVector GetVertexNormalByIndex (int i);

	// AMyKeypointDetector_Harris harris;
};
