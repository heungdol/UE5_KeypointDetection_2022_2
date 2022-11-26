#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include "../MyUtil/Types.h"
#include "Harris3D/MyUtil/Mesh.h"
#include "Harris3D/MyUtil/VertexType.h"
#define HKS 0
#define FAST_HKS 1
#define WKS 2
#define CURVE 3

using namespace std;

class Descriptor_HKS {
public:
    // constructor
	Descriptor_HKS() : mesh{nullptr} {}
    Descriptor_HKS(Mesh *mesh0, float t, int depth): mesh(mesh0), m_t(t), m_depth(depth) {}
	~Descriptor_HKS() {}

    // compute
	void compute(int descriptor);
                 // const std::string& eigFilename,
                 // const std::string& outFilename);
    
private:
    // compute eigenvalues and eigenvectors
    void computeEig(int K);//, const std::string& eigFilename);
    
    // compute hks
    void computeHks();
	
    // compute curvature descriptor
    void computeCurve();
    
    // normalize
    void normalize();
    
   
    Eigen::VectorXd evals;
    Eigen::MatrixXd evecs;

public:
	void InitKeypoints (std::vector<int>&, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);

	virtual string GetDetectorName()
	{
		return "========== Heat Kernel Signature ==========";
	};

	float m_t;
	int m_depth;
	// member variable
	Mesh *mesh;

	virtual void PrintDetectionInfo ()
	{
		// 출력
		// 이름
		// 파라미터
		// 전체 Keypoint 개수
		// Normal 별 개수
		// Type 별 개수

		cout << std::endl;
		cout << GetDetectorName () << std::endl;
		cout << std::endl;
		cout << "m_t: " << m_t << std::endl;
		cout << "m_depth: " << m_depth << std::endl;

		// cout << std::endl;
		// cout << "Mesh Info: " << *m_pMeshCom->GetName() << std::endl;
		// cout << "Area: " << myMesh->meshData.GetArea() << "(m^2)"<< std::endl;
		
		cout << std::endl;
		cout << "Total Keypoint Number: " << mesh->meshData.positions.size() << std::endl;

		int countUp = 0;
		int countParallel = 0;
		int countDown = 0;

		int countBump = 0;
		int countFlat = 0;
		int countSink = 0;

		// 노멀 구분하기
		// for (EVertexType type : vrtTypes_postSelected)
		// {
		// 	switch (type)
		// 	{
		// 	default:
		// 		break;
		// 	case EVertexType::VERTEX_BUMP:
		// 		countBump++;
		// 		break;
		// 	case EVertexType::VERTEX_FLAT:
		// 		countFlat++;
		// 		break;
		// 	case EVertexType::VERTEX_SINK:
		// 		countSink++;
		// 		break;
		// 	}
		// }
		//
		// for (EVertexNormalType type : vrtNorTypes_postSelected)
		// {
		// 	switch (type)
		// 	{
		// 	default:
		// 		break;
		// 	case EVertexNormalType::VERTEX_UP:
		// 		countUp++;
		// 		break;
		// 	case EVertexNormalType::VERTEX_PARALLEL:
		// 		countParallel++;
		// 		break;
		// 	case EVertexNormalType::VERTEX_DOWN:
		// 		countDown++;
		// 		break;
		// 	}
		// }
		//
		// cout << "Normal Number (Bump): " << countBump << std::endl;
		// cout << "Normal Number (Flat): " << countFlat << std::endl;
		// cout << "Normal Number (Sink): " << countSink << std::endl;
		//
		// cout << "Normal Number (Up): " << countUp << std::endl;
		// cout << "Normal Number (Parallel): " << countParallel << std::endl;
		// cout << "Normal Number (Down): " << countDown << std::endl;
	}
};

#endif
