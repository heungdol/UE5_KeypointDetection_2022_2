#pragma once

#include "MyType_HKS.h"

class MyEdge_HKS {
public:
	// one of the two half edges associated with this edge
	MyHalfEdgeIter_HKS he;
    
	// id between 0 and |E|-1
	int index;
    
	// checks if this edge lies on boundary
	bool isBoundary() const;
    
	double length() const;
};
