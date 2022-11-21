// Fill out your copyright notice in the Description page of Project Settings.


#include "MyKeypointDetector.h"

// Sets default values
AMyKeypointDetector::AMyKeypointDetector()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_pMeshCom = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MESH"));
	RootComponent = m_pMeshCom;
}

void AMyKeypointDetector::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
}

// Called when the game starts or when spawned
void AMyKeypointDetector::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AMyKeypointDetector::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AMyKeypointDetector::InitKeypointDetection()
{
}

void AMyKeypointDetector::InitSelectedVertexLocation()
{
}

FVector AMyKeypointDetector::GetCurrentVertexLocationByIndex(int i)
{
	if (i >= currentVrtLocs_postSelected.Num())
		return FVector (0, 0, 0);
		
	return  currentVrtLocs_postSelected [i];
}

FVector AMyKeypointDetector::GetCurrentVertexNormalByIndex(int i)
{
	if (i >= currentVrtLocs_postSelected.Num())
		return FVector (0, 0, 1);
		
	return  currentVrtNors_postSelected [i];
}


void AMyKeypointDetector::UpdateSelectedVertexLocation()
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
					FColor color = FColorList::Orange;
					DrawDebugBox(GetWorld()
						, currentVrtLocs_postSelected[i], FVector(1, 1, 1), FQuat::Identity
						, color, false, 0.1, 0, 1);

					/*switch (vrtTypes_postSelected[i])
					{
						default:
							break;
					case EVertexType::VERTEX_BUMP:
						color = FColorList::Blue;
						break;
					case EVertexType::VERTEX_FLAT:
						color = FColorList::Red;
						break;
					case EVertexType::VERTEX_SINK:
						color = FColorList::Green;
						break;
					}*/
					
					/*if (vrtNorTypes_postSelected[i] == EVertexNormalType::VERTEX_PARALLEL)
					{
						DrawDebugBox(GetWorld()
						, currentVrtLocs_postSelected[i], FVector(1, 1, 1), FQuat::Identity
						, color, false, 0.1, 0, 1);
					}
					else
					{
						DrawDebugLine(GetWorld()
						, currentVrtLocs_postSelected[i], currentVrtLocs_postSelected[i]+4*currentVrtNors_postSelected[i]
						, color, false, 0.1, 0, 1);
					}*/
				}
			}

			actorLocation = GetActorLocation();
			actorScale = GetActorScale();
			actorRotation = GetActorRotation();

		GetWorld()->GetTimerManager().ClearTimer(debugDrawWaitHandle);
	}), WaitTime, true); //반복도 여기서 추가 변수를 선언해 설정가능
}
