// Fill out your copyright notice in the Description page of Project Settings.


#include "MyMeshAnalysis.h"

// Sets default values
AMyMeshAnalysis::AMyMeshAnalysis()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_pMeshCom = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MESH"));
}

void AMyMeshAnalysis::InitSelectedVertexLocation()
{
	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		vrtLocs_postSelected.Push(meshData.GetVertexLocByIndex(vrts_postSelected[i]));
		currentVrtLocs_postSelected.Push(meshData.GetVertexLocByIndex(vrts_postSelected[i]));
		
		vrtNors_postSelected.Push (meshData.GetVertexNorByIndex (vrts_postSelected[i]));
		currentVrtNors_postSelected.Push (meshData.GetVertexNorByIndex (vrts_postSelected[i]));
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
}

void AMyMeshAnalysis::UpdateSelectedVertexLocation()
{
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
						, m_debugColor, false, 0.1, 0, 1);
				}

			}

		if (m_voxelOn)
		{
			// 바운딩박스
			DrawDebugBox(GetWorld()
						, m_boundingBoxCoord_pivot+actorLocation, (m_boundingBoxCoord_max-m_boundingBoxCoord_min)*0.5
						, FColorList::Blue, false, 0.1, 0, 5);
			
			for (int z = 0; z < voxelGridNum_z; z++)
			{
				for (int y = 0; y < voxelGridNum_y; y++)
				{
					for (int x = 0; x < voxelGridNum_x; x++)
					{
						// if (keypointNumberInVoxel [x + voxelGridNum_x * y + (voxelGridNum_x*voxelGridNum_y) * z] <= 0)
						// 	continue;

						if (keypointExistInVoxel [x + voxelGridNum_x * y + (voxelGridNum_x*voxelGridNum_y) * z] == false)
							continue;

						FColor trueColor = m_voxelFalseColor;

						if (keypointNumberInVoxel [x + voxelGridNum_x * y + (voxelGridNum_x*voxelGridNum_y) * z] >= m_voxelAbleThreshold)
							trueColor = m_voxelAbleColor;
						
						FVector center = voxelGridPivots [x + voxelGridNum_x * y + (voxelGridNum_x*voxelGridNum_y) * z] ;
						center += actorLocation;
						
						FVector extend = FVector(0.5*m_voxelGridSize);
						extend *= 0.9;
						
						DrawDebugBox(GetWorld()
						, center, extend
						, trueColor, false, 0.1, 0, 5);
					}
				}
			}
		}

			actorLocation = GetActorLocation();
			actorScale = GetActorScale();
			actorRotation = GetActorRotation();
		
		GetWorld()->GetTimerManager().ClearTimer(debugDrawWaitHandle);
	}), WaitTime, true); //반복도 여기서 추가 변수를 선언해 설정가능
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

		// 분석
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
			m_boundingBoxCoord_min = meshData.GetBoundingBoxCoord_Min();
			m_boundingBoxCoord_max = meshData.GetBoundingBoxCoord_Max();
			
			m_boundingBoxCoord_pivot = m_boundingBoxCoord_max + m_boundingBoxCoord_min;
			m_boundingBoxCoord_pivot /= 2.0;
			
			m_meshHeight = m_boundingBoxSize.Z;
			// m_meshObjectNumber = m_pMeshCom->section;
		}

		// 메쉬 초기화
		if (keypointDetectionBundle.InitMesh(m_pMeshCom, &meshData) == false)
			return;

		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		std::vector<int>{}.swap(vrts_selected);
		
		vrts_postSelected.Empty();
		vrtLocs_postSelected.Empty();
		vrtNors_postSelected.Empty();
		currentVrtLocs_postSelected.Empty();
		currentVrtNors_postSelected.Empty();
		vrtTypes_postSelected.Empty();

		switch (m_detectorType)
		{
		case EDetectorType::NONE:
			return;
			
		case EDetectorType::DT_HR:
			keypointDetectionBundle.SetParameters_Harris (m_ringSize, m_fraction, m_k);
			keypointDetectionBundle.InitKeypoints_Harris(vrts_selected, vrts_postSelected, vrtLocs_postSelected, vrtNors_postSelected
				, vrtTypes_postSelected, vrtNorTypes_postSelected);
			break;
		case EDetectorType::DT_HKS:
			keypointDetectionBundle.SetParameters_HKS (m_t, m_depth);
			keypointDetectionBundle.InitKeypoints_HKS(vrts_selected, vrts_postSelected, vrtLocs_postSelected, vrtNors_postSelected
				, vrtTypes_postSelected, vrtNorTypes_postSelected);
			break;
		case EDetectorType::DT_ISS:
			keypointDetectionBundle.SetParameters_ISS (m_saliencyRaidus, m_maxRadius, m_gamma_21, m_gamma_32, m_minNeighbors);
			keypointDetectionBundle.InitKeypoints_ISS(vrts_selected, vrts_postSelected, vrtLocs_postSelected, vrtNors_postSelected
				, vrtTypes_postSelected, vrtNorTypes_postSelected);
			break;
		case EDetectorType::DT_MS:
			keypointDetectionBundle.SetParameters_MeshSaliency(m_cutoffSaliency);
			keypointDetectionBundle.InitKeypoints_MeshSaliency(vrts_selected, vrts_postSelected, vrtLocs_postSelected, vrtNors_postSelected
				, vrtTypes_postSelected, vrtNorTypes_postSelected);
			break;
		}

		// TODO 각도에 따라 걸러내기
		// ...

		InitSelectedVertexLocation ();
		UpdateSelectedVertexLocation();

		// 복셀
		if (m_voxelOn)
		{
			voxelGridNum_x = UE4::SSE::CeilToInt32(m_boundingBoxSize.X / m_voxelGridSize);
			voxelGridNum_y = UE4::SSE::CeilToInt32(m_boundingBoxSize.Y / m_voxelGridSize);
			voxelGridNum_z = UE4::SSE::CeilToInt32(m_boundingBoxSize.Z / m_voxelGridSize);

			cout << voxelGridNum_x << endl << voxelGridNum_y << endl << voxelGridNum_z << endl;

			keypointNumberInVoxel.clear();
			vector<int>{}.swap(keypointNumberInVoxel);
			keypointNumberInVoxel = vector<int>(voxelGridNum_x*voxelGridNum_y*voxelGridNum_z);

			keypointExistInVoxel.clear();
			vector<bool>{}.swap(keypointExistInVoxel);
			keypointExistInVoxel = vector<bool>(voxelGridNum_x*voxelGridNum_y*voxelGridNum_z);

			voxelGridPivots.clear ();
			vector<FVector>{}.swap(voxelGridPivots);
			voxelGridPivots = vector<FVector>(voxelGridNum_x*voxelGridNum_y*voxelGridNum_z);

			double midX = voxelGridNum_x/2.0;
			double midY = voxelGridNum_y/2.0;
			double midZ = voxelGridNum_z/2.0;
			
			for (int z = 0; z < voxelGridNum_z; z++)
			{
				for (int y = 0; y < voxelGridNum_y; y++)
				{
					for (int x = 0; x < voxelGridNum_x; x++)
					{
						voxelGridPivots [x + voxelGridNum_x * y + (voxelGridNum_x*voxelGridNum_y) * z]
						= m_boundingBoxCoord_pivot + FVector (x - midX + 0.5, y - midY + 0.5, z - midZ + 0.5) * m_voxelGridSize;
					}
				}
			}

			
			// offsetX = (voxelGridNum_x%2 == 0) ? .5 : .5;
			// offsetY = (voxelGridNum_y%2 == 0) ? .5 : .5;
			// offsetZ = (voxelGridNum_z%2 == 0) ? .5 : .5;
			

			for (int i = 0; i < meshData.positions.size(); i++)
			{
				FVector vertPos = meshData.GetVertexLocByIndex(i);
				vertPos -= m_boundingBoxCoord_pivot;
				vertPos += FVector(midX - 0.5, midY - 0.5, midZ - 0.5) * m_voxelGridSize;
				vertPos /= m_voxelGridSize;

				int coordX = UE4::SSE::RoundToInt32(vertPos.X);
				int coordY = UE4::SSE::RoundToInt32(vertPos.Y);
				int coordZ = UE4::SSE::RoundToInt32(vertPos.Z);

				if (coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ < 0
					|| coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ >= keypointExistInVoxel.size())
				{
					cout << "Warning!" << endl;
					continue;
				}

				keypointExistInVoxel [coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ] = true;
			}

			for (int i = 0; i < vrts_selected.size(); i++)
			{
				FVector vertPos = meshData.GetVertexLocByIndex(vrts_selected[i]);
				vertPos -= m_boundingBoxCoord_pivot;
				vertPos += FVector(midX - 0.5, midY - 0.5, midZ - 0.5) * m_voxelGridSize;
				vertPos /= m_voxelGridSize;

				int coordX = UE4::SSE::RoundToInt32(vertPos.X);
				int coordY = UE4::SSE::RoundToInt32(vertPos.Y);
				int coordZ = UE4::SSE::RoundToInt32(vertPos.Z);

				if (coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ < 0
					|| coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ >= keypointExistInVoxel.size())
				{
					cout << "Warning!" << endl;
					continue;
				}

				keypointNumberInVoxel [coordX + voxelGridNum_x * coordY + (voxelGridNum_x * voxelGridNum_y) * coordZ]++;
			}
			
		}
	}
}

// Called when the game starts or when spawned
void AMyMeshAnalysis::BeginPlay()
{
	Super::BeginPlay();

	UpdateSelectedVertexLocation();
}

// Called every frame
void AMyMeshAnalysis::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

