#pragma once

#include "MyType_HKS.h"

class Index_HKS {
public:
	Index_HKS() {}
	Index_HKS(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
	bool operator<(const Index_HKS& i) const {
		if (position < i.position) return true;
		if (position > i.position) return false;
		if (uv < i.uv) return true;
		if (uv > i.uv) return false;
		if (normal < i.normal) return true;
		if (normal > i.normal) return false;
        
		return false;
	}
    
	int position;
	int uv;
	int normal;
};

class MeshData_HKS {
public:
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Vector3d> uvs;
	std::vector<Eigen::Vector3d> normals;
	std::vector<std::vector<Index_HKS>> indices;
};

class MyMeshIO_HKS {
public:
	// reads data from obj file
	static bool read(const UStaticMeshComponent* sm, MyMesh_HKS& mesh);
	
	// builds the halfedge mesh
	static bool buildMesh(const MeshData_HKS& data, MyMesh_HKS& mesh);
    
private:
	// reserves spave for mesh vertices, uvs, normals and faces
	static void preallocateMeshElements(const MeshData_HKS& data, MyMesh_HKS& mesh);
    
	// sets index for elements
	static void indexElements(MyMesh_HKS& mesh);
    
	// checks if any vertex is not contained in a face
	static void checkIsolatedVertices(const MyMesh_HKS& mesh);
    
	// checks if a vertex is non-manifold
	static void checkNonManifoldVertices(const MyMesh_HKS& mesh);
};