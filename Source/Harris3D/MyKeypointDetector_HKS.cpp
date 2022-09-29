// Fill out your copyright notice in the Description page of Project Settings.


#include "MyKeypointDetector_HKS.h"

#include "Spectra/MatOp/SparseSymMatProd.h"
#include "Spectra/MatOp/SparseCholesky.h"
#include "Spectra/SymGEigsSolver.h"

#define N 10

void AMyKeypointDetector_HKS::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 첫 실행 혹은 에디터에서 갱신 bool를 활성화할 시
	if (m_update_first == false || m_update_click == true)
	{
		// test
		/*if (m_update_first == false)
			cout << "first init" << endl;
		if (m_update_click == true)
			cout << "click to init" << endl;*/
		
		m_update_first = true;
		m_update_click = false;

		// 콜리젼 설정
		m_pMeshCom->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

		vrts_selected.clear();
		vrts_postSelected.Empty();

		vrtLocs_postSelected.Empty();
		vrtNors_postSelected.Empty();
		
		currentVrtLocs_postSelected.Empty();
		currentVrtNors_postSelected.Empty();
		
		InitKeypointDetection ();
		
		InitSelectedVertexLocation ();
		UpdateSelectedVertexLocation();
	}
}

void AMyKeypointDetector_HKS::BeginPlay()
{
	Super::BeginPlay();

	UpdateSelectedVertexLocation();
}

void AMyKeypointDetector_HKS::InitKeypointDetection()
{
	InitHKS ();
	ComputeHKS();

	if (mesh == NULL || mesh->GetIsEnableModel() == false)
		return;

	// 초기화 후 채워 넣기
	vrts_postSelected.Empty();
	
	for (MyVertexCIter_HKS v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
		if (v->isFeature(m_t, N)) vrts_postSelected.Push(v->index);// featureMap[v->index] = true;
		//else featureMap[v->index] = false;
	}
}

void AMyKeypointDetector_HKS::InitSelectedVertexLocation()
{
	actorLocation = GetActorLocation();
	actorScale = GetActorScale();
	actorRotation = GetActorRotation();
	
	// 선택된 점 위치 확인
	for (int i = 0; i < vrts_postSelected.Num(); i++)
	{
		vrtLocs_postSelected.Push(mesh->GetVertexLocByIndex(vrts_postSelected[i]));
		currentVrtLocs_postSelected.Push(mesh->GetVertexLocByIndex(vrts_postSelected[i]));
		
		vrtNors_postSelected.Push (mesh->GetVertexNorByIndex (vrts_postSelected[i]));
		currentVrtNors_postSelected.Push (mesh->GetVertexNorByIndex (vrts_postSelected[i]));
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

void AMyKeypointDetector_HKS::UpdateSelectedVertexLocation()
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
							, FColorList::Red, false, 0.1, 0, 1);
					}
				}

				actorLocation = GetActorLocation();
				actorScale = GetActorScale();
				actorRotation = GetActorRotation();

			GetWorld()->GetTimerManager().ClearTimer(debugDrawWaitHandle);
		}), WaitTime, true); //반복도 여기서 추가 변수를 선언해 설정가능
}

void AMyKeypointDetector_HKS::InitHKS()
{
	mesh = new MyMesh_HKS (m_pMeshCom);
}

void buildAdjacency(MyMesh_HKS *mesh, Eigen::SparseMatrix<double>& W)
{
	std::vector<Eigen::Triplet<double>> WTriplets;
    
	for (MyVertexCIter_HKS v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        
		MyHalfEdgeCIter_HKS he = v->he;
		double sumCoefficients = 0.0;
		do {
			double coefficient = 0.5 * (he->cotan() + he->flip->cotan());
			if (coefficient < 0.0) coefficient = 0.0;
			sumCoefficients += coefficient;
            
			WTriplets.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, -coefficient));
            
			he = he->flip->next;
		} while (he != v->he);
        
		WTriplets.push_back(Eigen::Triplet<double>(v->index, v->index,
												   sumCoefficients + 1e-8));
	}
    
	W.setFromTriplets(WTriplets.begin(), WTriplets.end());
}

void buildAreaMatrix(MyMesh_HKS *mesh, Eigen::SparseMatrix<double>& A, const double scale)
{
	std::vector<Eigen::Triplet<double>> ATriplets;
    
	double sum = 0.0;
	for (MyVertexCIter_HKS v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
		double area = v->dualArea();
		ATriplets.push_back(Eigen::Triplet<double>(v->index, v->index, area));
		sum += area;
	}
    
	A.setFromTriplets(ATriplets.begin(), ATriplets.end());
	A *= scale/sum;
}

int operator*(int Lhs, const FKey& Key);

void AMyKeypointDetector_HKS::ComputeHKS()
{
	// TODO
	
	if (mesh == NULL || mesh->GetIsEnableModel() == false)
		return;

	int _K = 500;
	int _v = (int)mesh->vertices.size();
        
	// build adjacency operator
	Eigen::SparseMatrix<double> W(_v, _v);
	buildAdjacency(mesh, W);
        
	// build area matrix
	Eigen::SparseMatrix<double> A(_v, _v);
	buildAreaMatrix(mesh, A, mesh->vertices.size());
        
	// compute eigenvectors and eigenvalues
	Spectra::SparseSymMatProd<double> opW(W);
	Spectra::SparseCholesky<double> opA(A);
        
	/*Spectra::SymGEigsSolver<
		Spectra::SparseSymMatProd<double>,
		Spectra::SparseCholesky<double>,
		Spectra::GEigsMode::Cholesky> geigs(&opW, &opA, _K, 2*_K);*/

	Spectra::SymGEigsSolver<double, Spectra::SMALLEST_MAGN
	, Spectra::SparseSymMatProd<double>, Spectra::SparseCholesky<double>
	, Spectra::GEIGS_CHOLESKY> geigs(&opW, &opA, _K, 2*_K);
        
	geigs.init();
	geigs.compute();

	evals = geigs.eigenvalues();
	evecs = geigs.eigenvectors();
        
	/*if (geigs.info() == Spectra::CompInfo::NotComputed) {
		evals = geigs.eigenvalues();
		evecs = geigs.eigenvectors();
            
	} else {
		std::cout << "Eigen computation failed" << std::endl;
		return;
	}*/


	
	int K = (int)evals.size();
	double ln = 4*log(10);
	double tmin = ln/evals(0);
	double step = (ln/evals(K-2) - tmin) / N;
    
	for (MyVertexIter_HKS v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
		v->descriptor = Eigen::VectorXd::Zero(N);
		Eigen::VectorXd C = Eigen::VectorXd::Zero(N);
        
		for (int j = K-2; j >= 0; j--) {
			double phi2 = evecs(v->index, j)*evecs(v->index, j);
			double t = tmin;
			double factor = 0.5;
            
			for (int i = 0; i < N; i++) {
				double exponent = exp(-evals(j)*t);
				v->descriptor(i) += phi2*exponent;
				C(i) += exponent;
				t += factor*step;
                
				// take larger steps with increasing t to bias ts towards high frequency features
				factor += 0.1;
			}
		}
        
		// normalize
		for (int i = 0; i < N; i++) {
			v->descriptor(i) /= C(i);
		}
	}
}
