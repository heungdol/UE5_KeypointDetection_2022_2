#pragma once

#define VERTEX_NUMBER_MAX 50000

#include "MyType_HKS.h"
#include "MyVertex_HKS.h"
#include "MyEdge_HKS.h"
#include "MyFace_HKS.h"
#include "MyHalfEdge_HKS.h"
#include <ThirdParty/Eigen/Eigen/SparseCore>

class MyMesh_HKS {
public:
	// default constructor
	MyMesh_HKS();
	MyMesh_HKS (const UStaticMeshComponent* sm);
	~MyMesh_HKS ();
        
	// // read mesh from file
	// bool read(const std::string& fileName);
 //    
	// // write mesh to file
	// bool write(const std::string& fileName) const;

	// member variables
	std::vector<MyHalfEdge_HKS> halfEdges;
	std::vector<MyVertex_HKS> vertices;
	std::vector<MyEdge_HKS> edges;
	std::vector<MyFace_HKS> faces;
	std::vector<MyHalfEdgeIter_HKS> boundaries;
	std::string name;
	bool enableModel = false;

	bool GetIsEnableModel ();

	FVector GetVertexLocByIndex(int ii);
	FVector GetVertexNorByIndex(int ii);

private:
	// center mesh about origin and rescale to unit radius
	void normalize();
};