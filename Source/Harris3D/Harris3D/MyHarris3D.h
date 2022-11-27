// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include <set>

#include "MeshDescription.h"
#include "MyMesh.h"
#include "../MyUtil/KeypointDetectionBundle.h"
#include "../MyUtil/VertexType.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyHarris3D.generated.h"

using namespace std;

// ================================================================================================

UCLASS()
class HARRIS3D_API AMyHarris3D : public AActor
{
	GENERATED_BODY()
	
public:
	virtual void OnConstruction(const FTransform& Transform) override;

	// Sets default values for this actor's properties
	AMyHarris3D();
	
	virtual string GetDetectorName()
	{
		return "Harris 3D";
	};

	virtual void PrintDetectionInfo ()
	{
		// 출력
		// 이름
		// 파라미터
		// 전체 Keypoint 개수
		// Normal 별 개수
		// Type 별 개수
		
	}

	/*UFUNCTION(BlueprintCallable)
	void UpdateHarris3D ();

	UFUNCTION(BlueprintCallable)
	bool GetIsUpdated ();*/
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	// ============================== default =======================================
	
	UPROPERTY (VisibleAnywhere)
	UStaticMeshComponent* m_pMeshCom;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_debugDraw = false;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_update_click = false;

	bool m_update_first = false;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	EDetectorType m_detectorType = EDetectorType::DT_HR;
	
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

	TArray <int> vrts_unselected;
	TArray <FVector> vrtLocs_unselected;
	TArray <FVector> vrtNors_unselected;
	TArray <FVector> currentVrtLocs_unselected;
	TArray <FVector> currentVrtNors_unselected;

	TArray <int> vrts_overlapped;
	TArray <FVector> vrtLocs_overlapped;
	TArray <FVector> vrtNors_overlapped;
	TArray <FVector> currentVrtLocs_overlapped;
	TArray <FVector> currentVrtNors_overlapped;
	
	FVector actorLocation;
	FVector actorScale;
	FRotator actorRotation;
	
	KeypointDetectionBundle keypointDetectionBundle;
	MeshData meshData;
	
	FVector GetVertexLocationByIndex (int i);
	FVector GetVertexNormalByIndex (int i);

	void InitSelectedVertexLocation ();
	void UpdateSelectedVertexLocation ();
};
