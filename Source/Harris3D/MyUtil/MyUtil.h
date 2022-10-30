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

class VertexNeighbor
{
public:
	int index;
	std::vector <int> neighborIndices = std::vector <int> {};

	/*void SetIndex (int i)
	{
		index = i;
	}

	void AddNeighbor (int i)
	{
		neighbors.push_back(i);
	}

	std::vector <int>& GetNeighbors ()
	{
		return neighbors;
	}*/
};


class MeshData {
public:
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Vector3d> uvs;
	std::vector<Eigen::Vector3d> normals;
	std::vector<std::vector<Index>> indices;   // 3개 단위
	std::vector <VertexNeighbor> neighbors;

	double GetArea ()
	{
		double ret = 0;

		for (std::vector<Index> index : indices)
		{
			Eigen::Vector3d v0 = positions[index[0].position];
			Eigen::Vector3d v1 = positions[index[1].position];
			Eigen::Vector3d v2 = positions[index[2].position];

			Eigen::Vector3d a = v1 - v0;
			Eigen::Vector3d b = v2 - v0;

			Eigen::Vector3d cross = a.cross(b);

			ret += abs (cross.norm()) * 0.5;
		}

		ret *= 0.01 * 0.01;
		return ret;
	}
};


class MyUtil
{
public:
	static bool ReadMeshWithoutOverwrap(const UStaticMeshComponent* sm, MeshData& meshData);
};
