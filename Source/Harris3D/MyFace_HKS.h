#pragma once

#include "MyType_HKS.h"
#include "MyBoundingBox_HKS.h"

class MyFace_HKS {
public:
	// one of the halfedges associated with this face
	MyHalfEdgeIter_HKS he;
    
	// id between 0 and |F|-1
	int index;
    
	// quasi conformal error
	Eigen::Vector3d qcError;
    
	// checks if this face lies on boundary
	//bool isBoundary() const;

	bool isBoundary = false;
    
	// returns normal to face
	//Eigen::Vector3d normal() const;
	Eigen::Vector3d normal;
    
	// returns face area
	double area() const;
    
	// returns centroid
	Eigen::Vector3d centroid() const;
    
	// returns bounding box
	MyBoundingBox_HKS boundingBox() const;
    
	// get nearest point on triangle
	double nearestPoint(Eigen::Vector3d& q, const Eigen::Vector3d& p) const;
};