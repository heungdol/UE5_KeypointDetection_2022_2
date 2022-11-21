// Fill out your copyright notice in the Description page of Project Settings.


#include "MyKeypointDetector_MeshSaliency.h"

void AMyKeypointDetector_MeshSaliency::OnConstruction(const FTransform& Transform)
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

		myMesh.Clear ();
		myMesh = Mesh (m_pMeshCom);
		descriptor = Descriptor_MeshSaliency (&myMesh, m_cutoffSaliency);

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

void AMyKeypointDetector_MeshSaliency::BeginPlay()
{
	Super::BeginPlay();

	UpdateSelectedVertexLocation();
}

void AMyKeypointDetector_MeshSaliency::InitKeypointDetection()
{
	Super::InitKeypointDetection();

	descriptor.InitKeypoints(vrts_selected, vrts_postSelected, vrtLocs_postSelected
		, vrtNors_postSelected, vrtTypes_postSelected, vrtNorTypes_postSelected);
}

void AMyKeypointDetector_MeshSaliency::InitSelectedVertexLocation()
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

		EVertexType vertexType = myMesh.vertices[vrts_postSelected[i]].getVertexType(myMesh.meshData, _dotFlat0, _dotFlat1, m_vertexType_depth);
		EVertexNormalType vertexNormalType = myMesh.vertices[vrts_postSelected[i]].getVertexNormalType(myMesh.meshData, _dotUp, _dotDown);
		vrtTypes_postSelected.Push(vertexType);
		vrtNorTypes_postSelected.Push(vertexNormalType);
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

