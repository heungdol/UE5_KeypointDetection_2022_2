#pragma once

#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <iostream>
#include "math.h"
#include "math.h"
#include <ThirdParty/Eigen/Eigen/Core>
#include <ThirdParty/Eigen/Eigen/Dense>
#include <ThirdParty/Eigen/Eigen/SparseCore>

class MyVertex_HKS;
class MyEdge_HKS;
class MyFace_HKS;
class MyHalfEdge_HKS;
class MyMesh_HKS;

typedef std::vector<MyHalfEdge_HKS>::iterator			MyHalfEdgeIter_HKS;
typedef std::vector<MyHalfEdge_HKS>::const_iterator		MyHalfEdgeCIter_HKS;
typedef std::vector<MyVertex_HKS>::iterator				MyVertexIter_HKS;
typedef std::vector<MyVertex_HKS>::const_iterator		MyVertexCIter_HKS;
typedef std::vector<MyEdge_HKS>::iterator				MyEdgeIter_HKS;
typedef std::vector<MyEdge_HKS>::const_iterator			MyEdgeCIter_HKS;
typedef std::vector<MyFace_HKS>::iterator				MyFaceIter_HKS;
typedef std::vector<MyFace_HKS>::const_iterator			MyFaceCIter_HKS;