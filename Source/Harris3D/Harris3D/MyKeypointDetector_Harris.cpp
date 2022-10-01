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

		// 만약 적절하지 않은 모델이라면
		// (버텍스 개수가 50000개 이상인 경우)
		if (myMesh.GetIsEnableModel() == false)
			return;	

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
	// 링 사이즈
	AMyKeypointDetector_Harris::ringSize = m_ringSize;

	// 전체 버택스 비율로 내어 선택할 개수를 구할 때 사용되는 상수
	AMyKeypointDetector_Harris::fraction_constant = m_fraction;
	
	AMyKeypointDetector_Harris::k_parameter = m_k;

	CalculateHarrisResponse ();
}

void AMyKeypointDetector_Harris::CalculateHarrisResponse()
{
	int vertexSize = myMesh.vertices.size();
	
	for (int indexVertex = 0; indexVertex < vertexSize; indexVertex++)
	{
		//vertexSize

		// 중복인 경우 계산하지 않고 컨티뉴
		if (indexVertex != myMesh.overlappingVert[indexVertex])
		{
			//harrisRPoints.push_back(harrisRPoints[myMesh.overlappingVert[indexVertex]]);
			harrisRPoints.push_back(1000);
			continue;
		}

		vector<double> x_coord, y_coord, z_coord;
		//caculate the neighbourhood
		set<int> set_nhd;

		//calculate the k rings neighbourhood of each vertex
		set_nhd = myMesh.CalculateNeighbourhood_Ring(indexVertex, ringSize);

		set<int>::iterator itr;
		for (itr = set_nhd.begin(); itr != set_nhd.end(); ++itr)
		{
			//get the x,y,z coordinates
			x_coord.push_back(myMesh.vertices[*itr].GetX());
			y_coord.push_back(myMesh.vertices[*itr].GetY());
			z_coord.push_back(myMesh.vertices[*itr].GetZ());
		}

		//adding the vertex itself to the set, the last element
		x_coord.push_back(myMesh.vertices[indexVertex].GetX());
		y_coord.push_back(myMesh.vertices[indexVertex].GetY());
		z_coord.push_back(myMesh.vertices[indexVertex].GetZ());


		//calculate centroid of the neighbourhood Vk(v)
		int nhd_size = x_coord.size();

		double sum_x = std::accumulate(x_coord.begin(), x_coord.end(), 0.0);
		double averg_x = (double)sum_x / nhd_size;

		double sum_y = std::accumulate(y_coord.begin(), y_coord.end(), 0.0);
		double averg_y = (double)sum_y / nhd_size;

		double sum_z = std::accumulate(z_coord.begin(), z_coord.end(), 0.0);
		double averg_z = (double)sum_z / nhd_size;

		//apply PCA to get the normal of the fitting plane
		//using Eigen Library

		//translate the set of points so that centroid is on the origin
		//Matrix= 3*nhd_size

		Eigen::MatrixXd nhd_matrix(3, nhd_size);
		for (int jj = 0; jj < nhd_size; jj++)
		{
			//store them in Matrix
			//x_trans = x_coord - x_centroid
			nhd_matrix(0, jj) = x_coord[jj] - averg_x;
			nhd_matrix(1, jj) = y_coord[jj] - averg_y;
			nhd_matrix(2, jj) = z_coord[jj] - averg_z;
		}

		//Covariance matrix C
		// 1/n-1*X*Xt
		Eigen::Matrix3d CovM;
		CovM = (nhd_matrix * nhd_matrix.transpose()) / (nhd_size - 1); //creates a symmetric matrix

		// Calculate EigenVectors and EigenValues of Covaraince matrix
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(CovM);
		//SelfAdjointEigenSolver if the matrix is symmetric(faster)

		Eigen::MatrixXd eig_values(3, 1);
		eig_values = es.eigenvalues().real(); //sorted in increasing order
		Eigen::Matrix3d principal_comps = es.eigenvectors().real();

		//sort eigenvectors in decreasing order by swaping
		Eigen::MatrixXd tmp(3, 1);
		tmp = principal_comps.col(0);
		principal_comps.col(0) = principal_comps.col(2);
		principal_comps.col(2) = tmp;


		//set of points is rotated so that the normal of the fitting plane is the z-axis
		Eigen::MatrixXd rotated_points(3, nhd_size);
		rotated_points = principal_comps.transpose() * nhd_matrix;

		//translate the set of points so that the point v is in the origin of the XY-plane
		double x_vertex = rotated_points(0, nhd_size - 1);
		double y_vertex = rotated_points(1, nhd_size - 1);
		double z_vertex = rotated_points(2, nhd_size - 1);

		Eigen::MatrixXd trans_points(3, nhd_size);
		for (int jk = 0; jk < nhd_size; jk++)
		{
			//trans_points = rotated_points - vertex
			trans_points(0, jk) = rotated_points(0, jk) - x_vertex;
			trans_points(1, jk) = rotated_points(1, jk) - y_vertex;
			trans_points(2, jk) = rotated_points(2, jk) - z_vertex;
		}

		//fit a quadratic surface to the set of transformed points
		//z = f(x,y) =p1/2*x2 +p2*x*y + p3/2*y2 +p4*x +p5*y +p6
		Eigen::MatrixXd eqns(nhd_size, 6); // equations
		Eigen::MatrixXd bvector(nhd_size, 1);
		Eigen::MatrixXd xvector(6, 1);
		for (int kk = 0; kk < nhd_size; kk++)
		{
			double xv = trans_points(0, kk);
			double yv = trans_points(1, kk);
			double zv = trans_points(2, kk);

			bvector(kk, 0) = zv;

			eqns(kk, 0) = (xv * xv) / 2; //coefficient of p1
			eqns(kk, 1) = xv * yv; //p2
			eqns(kk, 2) = (yv * yv) / 2; //p3
			eqns(kk, 3) = xv; //p4
			eqns(kk, 4) = yv; //p5
			eqns(kk, 5) = 1; //p6
		}

		//solve the linear system Ax=b
		xvector = eqns.colPivHouseholderQr().solve(bvector);

		//extract the solution of the linear system
		double p1 = xvector(0, 0);
		double p2 = xvector(1, 0);
		double p3 = xvector(2, 0);
		double p4 = xvector(3, 0);
		double p5 = xvector(4, 0);
		double p6 = xvector(5, 0);

		double A = p4 * p4 + 2 * p1 * p1 + 2 * p2 * p2;
		double B = p4 * p4 + 2 * p2 * p2 + 2 * p3 * p3; //difference with source code p5 = p2 =0.3..
		double C = p4 * p5 + 2 * p1 * p2 + 2 * p2 * p3;

		//Harris operator value in the point v        
		double harrisV = (A * B) - (C * C) - k_parameter * ((A + B) * (A + B));
		harrisRPoints.push_back(harrisV);
	} //endforeachvertex

	//Pre-selection of the interest points
	//preserve the vertices which are local maximum
	// 주변 이웃한 버텍스 수가 가장 많은 버텍스
	vector<int> preselected;
	for (int nV = 0; nV < vertexSize; nV++)
	{
		// 중복 패스
		if (nV != myMesh.overlappingVert[nV])
		{
			continue;
		}
		
		bool localMaxima = GetIsLocalMaxima(nV);
		if (localMaxima == true)
		{
			preselected.push_back(nV);
		}
	}
	//sort the preselected vertices, decreasing order
	sort(preselected.rbegin(), preselected.rend());
	{
		//Selecting interest points
		vector<int> selectedVertices; //Highest Harris Method

		//Convert set to VectorXi
		int numPreselected = preselected.size();
		Eigen::VectorXi preSelectedVertexes(numPreselected);
		int ctrlVar1(0);
		for (vector<int>::iterator it = preselected.begin(); it != preselected.end(); ++it)
		{
			preSelectedVertexes(ctrlVar1) = *it;
			ctrlVar1++;
		}

		//Get vector with harris values
		Eigen::VectorXd preSelectedHarrisValues(numPreselected);
		for (int iPre = 0; iPre < numPreselected; iPre++)
		{
			preSelectedHarrisValues(iPre) = harrisRPoints[preSelectedVertexes(iPre)];
		}

		vector<int> _selectedVertices;

		double maxi(0);
		for (int iIP = 0; iIP < preSelectedVertexes.size(); iIP++)
		{
			maxi = preSelectedHarrisValues.maxCoeff();
			for (int i = 0; i < preSelectedVertexes.size(); i++)
			{
				if (abs(maxi - preSelectedHarrisValues(i)) < 0.00001)
				{
					_selectedVertices.push_back(preSelectedVertexes(i));
					preSelectedHarrisValues(i) = 0;
					break;
				}
			}
		}

		//sort the preselected vertices, decreasing order
		sort(preselected.rbegin(), preselected.rend());

		vrts_selected = _selectedVertices;
	}

	// 정렬 및 중복 제거
	sort (vrts_selected.begin(), vrts_selected.end());
	vrts_selected.erase (unique (vrts_selected.begin(), vrts_selected.end()), vrts_selected.end());

	// ...
	for (int i = 0; i < vrts_selected.size(); i++)
		vrts_postSelected.Add(vrts_selected[i]);
}

bool AMyKeypointDetector_Harris::GetIsLocalMaxima(unsigned int vertexIndex)
{
	set<int> nhd = myMesh.vertices[vertexIndex].GetNeighbours();
	set<int>::iterator itrr;
	for (itrr = nhd.begin(); itrr != nhd.end(); ++itrr)
	{
		if (harrisRPoints[vertexIndex] < harrisRPoints[*itrr])
		{
			return false;
		}
	}
	
	return true;
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