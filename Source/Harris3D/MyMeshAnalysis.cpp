// Fill out your copyright notice in the Description page of Project Settings.


#include "MyMeshAnalysis.h"

// Sets default values
AMyMeshAnalysis::AMyMeshAnalysis()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_pMeshCom = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MESH"));
}

void AMyMeshAnalysis::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 첫 실행 혹은 에디터에서 갱신 bool를 활성화할 시
	if (m_update_first == false || m_update_click == true)
	{
		m_update_first = true;
		m_update_click = false;
		
		// 메쉬 판단
		if (!m_pMeshCom)
			return;

		meshData.Clear();
		if (MyUtil::ReadMeshWithoutOverwrap(m_pMeshCom, meshData))
		{
			m_vertexNumber = meshData.GetTotalVertexNumber();
			m_surfaceArea = meshData.GetArea();
			m_vertexRatioByArea = (m_surfaceArea != 0) ? (m_vertexNumber * 1.0 / m_surfaceArea) : 0;

			m_vertexNumber_valid = meshData.GetTotalVertexNumber_Valid();
			m_surfaceArea_valid = meshData.GetArea_Valid();
			m_vertexRatioByArea_valid = (m_surfaceArea_valid != 0) ? (m_vertexNumber_valid * 1.0 / m_surfaceArea_valid) : 0;
			
			m_boundingBoxSize = meshData.GetBoundingBoxSize();
			m_meshHeight = m_boundingBoxSize.Z;
			// m_meshObjectNumber = m_pMeshCom->section;
		}
	}
}

// Called when the game starts or when spawned
void AMyMeshAnalysis::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AMyMeshAnalysis::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

