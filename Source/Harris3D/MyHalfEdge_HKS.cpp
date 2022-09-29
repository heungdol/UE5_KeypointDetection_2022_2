#include "MyHalfEdge_HKS.h"
#include "MyVertex_HKS.h"

double MyHalfEdge_HKS::cotan() const
{
	if (onBoundary) return 0.0;
    
	Eigen::Vector3d p0 = vertex->position;
	Eigen::Vector3d p1 = next->vertex->position;
	Eigen::Vector3d p2 = next->next->vertex->position;
    
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p2 - p0;
    
	return v1.dot(v2) / v1.cross(v2).norm();
}