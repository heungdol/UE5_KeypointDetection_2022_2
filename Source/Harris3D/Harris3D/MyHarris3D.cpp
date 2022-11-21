// Fill out your copyright notice in the Description page of Project Settings.

#include "MyHarris3D.h"

#include <algorithm> //for difference
#include <iostream>
#include <iterator>
#include <numeric> //for sum
#include <set>
#include <string>

#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Dense>
#include <ThirdParty/Eigen/Eigen/Eigenvalues>

#include "Containers/UnrealString.h"

using namespace std;
using namespace ECollisionEnabled;

// Construction
void AMyHarris3D::OnConstruction(const FTransform& Transform)
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

		switch (m_detectorType)
		{
		case EDetectorType::DT_HARRIS3D:

			break;
		case EDetectorType::DT_HKS:

			break;
		case EDetectorType::DT_ISS:

			break;
		case EDetectorType::DT_MESHSALIENCY:

			break;
		}

		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		vector <int>().swap(vrts_selected);
		
		vrts_postSelected.Empty();
		
		vrtLocs_postSelected.Empty();
		vrtNors_postSelected.Empty();

		vrtTypes_postSelected.Empty();
		// vrtNorTypes_postSelected.Empty();
		
		currentVrtLocs_postSelected.Empty();
		currentVrtNors_postSelected.Empty();
	}
}

/*
bool AMyHarris3D::GetIsUpdated()
{
	return m_update_first;
}

void AMyHarris3D::UpdateHarris3D()
{
	
}
*/


// Sets default values
AMyHarris3D::AMyHarris3D()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_pMeshCom = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MESH"));
	RootComponent = m_pMeshCom;
}



// Called when the game starts or when spawned
void AMyHarris3D::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AMyHarris3D::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

FVector AMyHarris3D::GetVertexLocationByIndex (int i)
{
	if (i >= currentVrtLocs_postSelected.Num())
		return FVector (0, 0, 0);
		
	return  currentVrtLocs_postSelected [i];
}


FVector AMyHarris3D::GetVertexNormalByIndex (int i)
{
	if (i >= currentVrtLocs_postSelected.Num())
		return FVector (0, 0, 1);
		
	return  currentVrtNors_postSelected [i];
}
	
