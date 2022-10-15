#pragma once
#include <ThirdParty/Eigen/Eigen/Core>
#include "KismetProceduralMeshLibrary.h"

class Index {
public:
	Index() {}
    
	Index(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
	bool operator<(const Index& i) const {
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

class MeshData {
public:
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Vector3d> uvs;
	std::vector<Eigen::Vector3d> normals;
	std::vector<std::vector<Index>> indices;   // 3개 단위
};


class MyUtil
{
public:
	static bool ReadMeshWithoutOverwrap(const UStaticMeshComponent* sm, MeshData& meshData);
};
