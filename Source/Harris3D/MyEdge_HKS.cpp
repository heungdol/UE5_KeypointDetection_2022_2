#include "MyEdge_HKS.h"
#include "MyHalfEdge_HKS.h"
#include "MyVertex_HKS.h"

double MyEdge_HKS::length() const
{
	Eigen::Vector3d a = he->vertex->position;
	Eigen::Vector3d b = he->flip->vertex->position;
    
	return (b-a).norm();
}

bool MyEdge_HKS::isBoundary() const
{
	return he->onBoundary || he->flip->onBoundary;
}
