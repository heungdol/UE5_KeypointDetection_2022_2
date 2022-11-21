// Fill out your copyright notice in the Description page of Project Settings.

#include "MyKeypointDetector_Harris.h"

#include <algorithm> //for difference
#include <numeric> //for sum
#include <set>

#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Dense>
#include <ThirdParty/Eigen/Eigen/Eigenvalues>

void AMyKeypointDetector_Harris::OnConstruction(const FTransform& Transform)
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
	
		myMesh = MyMesh (m_pMeshCom);
		myDescriptor = Descriptor_Harris3D (&myMesh, m_ringSize, m_fraction, m_k);

		// 만약 적절하지 않은 모델이라면
		// (버텍스 개수가 50000개 이상인 경우)
		if (myMesh.GetIsEnableModel() == false)
			return;	

		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		vector <int>().swap(vrts_selected);
		
		vrts_postSelected.Empty();
		
		vrtLocs_postSelected.Empty();
		vrtNors_postSelected.Empty();

		vrtTypes_postSelected.Empty();
		vrtNorTypes_postSelected.Empty();
		
		currentVrtLocs_postSelected.Empty();
		currentVrtNors_postSelected.Empty();
		
		InitKeypointDetection ();

		InitSelectedVertexLocation ();
		UpdateSelectedVertexLocation();
	}
}

void AMyKeypointDetector_Harris::BeginPlay()
{
	Super::BeginPlay();

	UpdateSelectedVertexLocation();
}

void AMyKeypointDetector_Harris::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AMyKeypointDetector_Harris::InitKeypointDetection()
{
	Super::InitKeypointDetection();

	myDescriptor.InitKeypoints(vrts_selected, vrts_postSelected, vrtLocs_postSelected
		, vrtNors_postSelected, vrtTypes_postSelected, vrtNorTypes_postSelected);
}

void AMyKeypointDetector_Harris::InitSelectedVertexLocation()
{
	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		vrtLocs_postSelected.Push(myMesh.GetVertexLocByIndex(vrts_postSelected[i]));
		currentVrtLocs_postSelected.Push(myMesh.GetVertexLocByIndex(vrts_postSelected[i]));
		
		vrtNors_postSelected.Push (myMesh.GetVertexNorByIndex (vrts_postSelected[i]));
		currentVrtNors_postSelected.Push (myMesh.GetVertexNorByIndex (vrts_postSelected[i]));

		vrtTypes_postSelected.Push(myMesh.vertices[vrts_postSelected[i]].GetVertexType(&myMesh, m_vertexType_depth, _dotFlat0, _dotFlat1));
		vrtNorTypes_postSelected.Push(myMesh.vertices[vrts_postSelected[i]].GetVertexNormalType(_dotUp, _dotDown));
	}

	for (int i = 0; i < vrtLocs_postSelected.Num(); i++)
	{
		//GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Yellow, TEXT("OHa"));

		FVector offset = vrtLocs_postSelected[i];
		offset *= actorScale;
		offset = actorRotation.RotateVector(offset);
		
		currentVrtLocs_postSelected [i] = actorLocation + offset;
		currentVrtNors_postSelected [i] = actorRotation.RotateVector(vrtNors_postSelected [i]);
	}

	PrintDetectionInfo ();
}