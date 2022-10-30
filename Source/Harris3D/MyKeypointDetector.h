// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyUtil/VertexType.h"
#include "MyKeypointDetector.generated.h"

using namespace  std;

UCLASS()
class HARRIS3D_API AMyKeypointDetector : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMyKeypointDetector();

	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	const float _dotDown = -0.5f;
	const float _dotUp = 0.5f;

	const float _dotFlat0 = -0.5f;
	const float _dotFlat1 = 0.5f;
	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual string GetDetectorName()
	{
		return "NONE";
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

	UPROPERTY (VisibleAnywhere)
	UStaticMeshComponent* m_pMeshCom;
	
	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_debugDraw = false;

	UPROPERTY(EditAnywhere, Category="Keypoint Detector: Default")
	bool m_update_click = false;
	
	bool m_update_first = false;

	FVector actorLocation;
	FVector actorScale;
	FRotator actorRotation;

	vector <int> vrts_selected;
	TArray <int> vrts_postSelected;
	// TArray <int> vrts_unselected;
	// TArray <int> vrts_overlapped;
	
	TArray <FVector> vrtLocs_postSelected;
	TArray <FVector> vrtNors_postSelected;
	TArray <EVertexType> vrtTypes_postSelected;
	TArray <EVertexNormalType> vrtNorTypes_postSelected;
	
	TArray <FVector> currentVrtLocs_postSelected;
	TArray <FVector> currentVrtNors_postSelected;
	
	// TArray <FVector> vrtLocs_unselected;
	// TArray <FVector> vrtNors_unselected;
	// TArray <FVector> currentVrtLocs_unselected;
	// TArray <FVector> currentVrtNors_unselected;
	
	virtual void InitKeypointDetection ();

	virtual void InitSelectedVertexLocation ();
	virtual void UpdateSelectedVertexLocation ();
	
	FVector GetCurrentVertexLocationByIndex (int i);
	FVector GetCurrentVertexNormalByIndex (int i);
};
