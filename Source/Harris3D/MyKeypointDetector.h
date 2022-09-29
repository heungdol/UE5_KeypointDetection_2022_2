// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
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
	// Called every frame
	virtual void Tick(float DeltaTime) override;

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
