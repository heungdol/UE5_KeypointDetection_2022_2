#pragma once

#include "MyType_HKS.h"

class MyHalfEdge_HKS {
public:
	// next halfedge around the current face
	MyHalfEdgeIter_HKS next;
    
	// other halfedge associated with this edge
	MyHalfEdgeIter_HKS flip;
    
	// vertex at the tail of the halfedge
	MyVertexIter_HKS vertex;
    
	// edge associated with this halfedge
	MyEdgeIter_HKS edge;
    
	// face associated with this halfedge
	MyFaceIter_HKS face;
    
	// id between 0 and |H|-1
	int index;
    
	// checks if this halfedge is contained in boundary loop
	bool onBoundary;
    
	// computes the cotan value
	double cotan() const;
};
