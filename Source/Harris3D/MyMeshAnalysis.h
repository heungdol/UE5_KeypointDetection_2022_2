// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MyUtil/MeshIO.h"
#include "MyMeshAnalysis.generated.h"

UCLASS()
class HARRIS3D_API AMyMeshAnalysis : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMyMeshAnalysis();

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
	
	// UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	// int m_meshObjectNumber = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	double m_meshHeight = 0;
	
	UPROPERTY(EditAnywhere, Category="Mesh Analysis")
	FVector m_boundingBoxSize = FVector::Zero();
};
