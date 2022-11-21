#include "MyKeypointDetector_ISS.h"
// Fill out your copyright notice in the Description page of Project Settings.

void AMyKeypointDetector_ISS::OnConstruction(const FTransform& Transform)
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
		myDescriptor.m_pMeshCom = m_pMeshCom;
		myDescriptor.meshData = &meshData;
		myDescriptor.m_saliencyRaidus = m_saliencyRaidus;
		myDescriptor.m_maxRadius = m_maxRadius;
		myDescriptor.m_gamma_21 = m_gamma_21;
		myDescriptor.m_gamma_32 = m_gamma_32;
		myDescriptor.m_minNeighbors = m_minNeighbors;
		
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

void AMyKeypointDetector_ISS::BeginPlay()
{
	Super::BeginPlay();
}

void AMyKeypointDetector_ISS::InitKeypointDetection()
{
	Super::InitKeypointDetection();

	// meshData.indices.clear();
	// meshData.normals.clear();
	// meshData.positions.clear();
	// meshData.uvs.clear();

	myDescriptor.InitKeypoints(vrts_selected, vrts_postSelected, vrtLocs_postSelected
		, vrtNors_postSelected, vrtTypes_postSelected, vrtNorTypes_postSelected);
}

void AMyKeypointDetector_ISS::InitSelectedVertexLocation()
{
	Super::InitSelectedVertexLocation();

	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		FVector pos;
		pos.X = meshData.positions[vrts_postSelected[i]][0];
		pos.Y = meshData.positions[vrts_postSelected[i]][1];
		pos.Z = meshData.positions[vrts_postSelected[i]][2];

		FVector no;
		no.X = meshData.normals[vrts_postSelected[i]][0];
		no.Y = meshData.normals[vrts_postSelected[i]][1];
		no.Z = meshData.normals[vrts_postSelected[i]][2];
		
		vrtLocs_postSelected.Add(pos);
		currentVrtLocs_postSelected.Add(pos);
		
		vrtNors_postSelected.Add(no);
		currentVrtNors_postSelected.Add(no);

		// vrtTypes_postSelected.Push(GetVertexType(vrts_postSelected[i], m_vertexType_depth, _dotFlat0, _dotFlat1));
		// vrtNorTypes_postSelected.Push(GetVertexNormalType(vrts_postSelected[i], _dotUp, _dotDown));
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

	PrintDetectionInfo();
}

/*
void AMyKeypointDetector_ISS::UpdateSelectedVertexLocation()
{
	Super::UpdateSelectedVertexLocation();

	FTimerHandle debugDrawWaitHandle;
	
	float WaitTime = 0.1; //시간 설정
	//bool  firstInit =  false;

	if (GetWorld())
		GetWorld()->GetTimerManager().SetTimer(debugDrawWaitHandle, FTimerDelegate::CreateLambda([&]()
		{
				for (int i = 0; i < vrtLocs_postSelected.Num(); i++)
				{
					//GEngine->AddOnScreenDebugMessage(-1, 2, FColor::Yellow, TEXT("OHa"));

					FVector offset = vrtLocs_postSelected[i];
					offset *= actorScale;
					offset = actorRotation.RotateVector(offset);
				
					currentVrtLocs_postSelected [i] = actorLocation + offset;
					currentVrtNors_postSelected [i] = actorRotation.RotateVector(vrtNors_postSelected [i]);

					if (m_debugDraw == true)
					{
						DrawDebugLine(GetWorld()
							, currentVrtLocs_postSelected[i], currentVrtLocs_postSelected[i]+4*currentVrtNors_postSelected[i]
							, FColorList::Red, false, 0.1, 0, 1);
					}
				}

				actorLocation = GetActorLocation();
				actorScale = GetActorScale();
				actorRotation = GetActorRotation();

			GetWorld()->GetTimerManager().ClearTimer(debugDrawWaitHandle);
		}), WaitTime, true); //반복도 여기서 추가 변수를 선언해 설정가능
}
*/
