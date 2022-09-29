#include "MyMesh_HKS.h"

#include "AITypes.h"
#include "KismetProceduralMeshLibrary.h"
#include "MyMeshIO_HKS.h"

MyMesh_HKS::MyMesh_HKS()
{
    
}

MyMesh_HKS::MyMesh_HKS(const UStaticMeshComponent* sm)
{
	enableModel = MyMeshIO_HKS::read(sm, *this);
}

extern std::vector<MyHalfEdge_HKS> isolated;

bool MyMesh_HKS::GetIsEnableModel()
{
	return enableModel;
}

FVector MyMesh_HKS::GetVertexLocByIndex(int ii)
{
	if (vertices.size() <= ii)
		return FVector (0, 0, 0);

	return FVector ( vertices [ii].position[0],  vertices [ii].position[1],  vertices [ii].position[2]);
}

FVector MyMesh_HKS::GetVertexNorByIndex(int ii)
{
	if (vertices.size() <= ii)
		return FVector (0, 0, 1);

	return FVector ( vertices [ii].normal()[0], vertices [ii].normal()[1], vertices [ii].normal()[2]);
}

/*
bool MyMesh_HKS::buildMesh(const MeshData_HKS& data)
{
	std::map<std::pair<int, int>, int> edgeCount;
    std::map<std::pair<int, int>, MyHalfEdgeIter_HKS> existingHalfEdges;
    std::map<int, MyVertexIter_HKS> indexToVertex;
    std::map<MyHalfEdgeIter_HKS, bool> hasFlipEdge;
    
    preallocateMeshElements(data);
    
    // insert vertices into mesh and map vertex indices to vertex pointers
    for (unsigned int i = 0; i < data.positions.size(); i++) {
        MyVertexIter_HKS vertex = vertices.insert(vertices.end(), MyVertex_HKS());
        vertex->position = data.positions[i];
        vertex->he = isolated.begin();
        indexToVertex[i] = vertex;
    }

	// for (int i = 0; i < data.normals.size(); i++)
	// 	normals.push_back(FVector(data.normals[i].x, data.normals[i].y(), data.normals[i].z));
 //   
    // insert faces into mesh
    int faceIndex = 0;
    bool degenerateFaces = false;
    for (std::vector<std::vector<Index_HKS>>::const_iterator f  = data.indices.begin();
                                                         f != data.indices.end();
                                                         f ++) {
        int n = (int)f->size();
        
        // check if face is degenerate
        if (n < 3) {
            std::cerr << "Error: face " << faceIndex << " is degenerate" << std::endl;
            degenerateFaces = true;
            continue;
        }
        
        // create face
        MyFaceIter_HKS newFace = faces.insert(faces.end(), MyFace_HKS());
        
        // create a halfedge for each edge of the face
        std::vector<MyHalfEdgeIter_HKS> _halfEdges(n);
        for (int i = 0; i < n; i++) {
            _halfEdges[i] = halfEdges.insert(halfEdges.end(), MyHalfEdge_HKS());
        }
        
        // initialize the halfedges
        for (int i = 0; i < n; i++) {
            // vertex indices
            int a = (*f)[i].position;
            int b = (*f)[(i+1)%n].position;
            
            // set halfedge attributes
            _halfEdges[i]->next = _halfEdges[(i+1)%n];
            _halfEdges[i]->vertex = indexToVertex[a];
            
            _halfEdges[i]->onBoundary = false;
            
            // keep track of which halfedges have flip edges defined (for deteting boundaries)
            hasFlipEdge[_halfEdges[i]] = false;
            
            // point vertex a at the current halfedge
            indexToVertex[a]->he = _halfEdges[i];
            
            // point new face and halfedge to each other
            _halfEdges[i]->face = newFace;
            newFace->he = _halfEdges[i];
            
            // if an edge between a and b has been created in the past, it is the flip edge of the current halfedge
            if (a > b) std::swap(a, b);
            if (existingHalfEdges.find(std::pair<int, int>(a, b)) != existingHalfEdges.end()) {
                _halfEdges[i]->flip = existingHalfEdges[std::pair<int, int>(a, b)];
                _halfEdges[i]->flip->flip = _halfEdges[i];
                _halfEdges[i]->edge = _halfEdges[i]->flip->edge;
                hasFlipEdge[_halfEdges[i]] = true;
                hasFlipEdge[_halfEdges[i]->flip] = true;
                
            } else {
                // create an edge and set its halfedge
                _halfEdges[i]->edge = edges.insert(edges.end(), MyEdge_HKS());
                _halfEdges[i]->edge->he = _halfEdges[i];
                edgeCount[std::pair<int, int>(a, b)] = 0;
            }
            
            // record that halfedge has been created from a to b
            existingHalfEdges[std::pair<int, int>(a, b)] = _halfEdges[i];
            
            // check for nonmanifold edges
            edgeCount[std::pair<int, int>(a, b)] ++;
            if (edgeCount[std::pair<int, int>(a, b)] > 2) {
                std::cerr << "Error: edge " << a << ", " << b << " is non manifold" << std::endl;
                return false;
            }
        }
        
        faceIndex++;
    }
    
    if (degenerateFaces) {
        return false;
    }
    
    // insert extra faces for boundary cycle
    for (MyHalfEdgeIter_HKS currHe = halfEdges.begin(); currHe != halfEdges.end(); currHe++) {
        // if a halfedge with no flip edge is found, create a new face and link it the corresponding boundary cycle
        if (!hasFlipEdge[currHe]) {
            // create face
            MyFaceIter_HKS newFace = faces.insert(faces.end(), MyFace_HKS());
            
            // walk along boundary cycle
            std::vector<MyHalfEdgeIter_HKS> boundaryCycle;
            MyHalfEdgeIter_HKS he = currHe;
            do {
                // create a new halfedge on the boundary face
                MyHalfEdgeIter_HKS newHe = halfEdges.insert(halfEdges.end(), MyHalfEdge_HKS());
                newHe->onBoundary = true;
                
                // link the current halfedge in the cycle to its new flip edge
                he->flip = newHe;
                
                // grab the next halfedge along the boundary by finding
                // the next halfedge around the current vertex that doesn't
                // have a flip edge defined
                MyHalfEdgeIter_HKS nextHe = he->next;
                while (hasFlipEdge[nextHe]) {
                    nextHe = nextHe->flip->next;
                }
                
                // set attritubes for new halfedge
                newHe->flip = he;
                newHe->vertex = nextHe->vertex;
                newHe->edge = he->edge;
                newHe->face = newFace;
                
                // set face's halfedge to boundary halfedge
                newFace->he = newHe;
                
                boundaryCycle.push_back(newHe);
                
                // continue walk along cycle
                he = nextHe;
                
            } while (he != currHe);
            
            // link the cycle of boundary halfedges together
            int n = (int)boundaryCycle.size();
            for (int i = 0; i < n; i++) {
                boundaryCycle[i]->next = boundaryCycle[(i+n-1)%n];
                hasFlipEdge[boundaryCycle[i]] = true;
                hasFlipEdge[boundaryCycle[i]->flip] = true;
            }
            boundaries.insert(boundaries.end(), boundaryCycle[0]);
        }
    }
    
    indexElements();
    checkIsolatedVertices();
    checkNonManifoldVertices();
    
    return true;
}

std::string stringRep(const Eigen::Vector3d& v)
{
	return std::to_string(v.x()) + " " + std::to_string(v.y()) + " " + std::to_string(v.z());
}

void MyMesh_HKS::checkNonManifoldVertices ()
{
	std::unordered_map<std::string, int> vertexFaceMap;
    
	for (MyFaceCIter_HKS f = faces.begin(); f != faces.end(); f++) {
		MyHalfEdgeCIter_HKS he = f->he;
		do {
			vertexFaceMap[stringRep(he->vertex->position)] ++;
			he = he->next;
            
		} while (he != f->he);
	}
    
	for (MyVertexCIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		int valence = 0;
		MyHalfEdgeCIter_HKS he = v->he;
		do {
			valence ++;
			he = he->flip->next;
            
		} while (he != v->he);
        
		if (vertexFaceMap[stringRep(v->position)] != valence) {
			std::cerr << "Warning: vertex " << v->index
					  << " is nonmanifold." << std::endl;
		}
	}
}

void MyMesh_HKS::indexElements ()
{
	int index = 0;
	for (MyVertexIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		v->index = index;
		index++;
	}
    
	index = 0;
	for (MyEdgeIter_HKS e = edges.begin(); e != edges.end(); e++) {
		e->index = index;
		index++;
	}
    
	index = 0;
	for (MyHalfEdgeIter_HKS h = halfEdges.begin(); h != halfEdges.end(); h++) {
		h->index = index;
		index++;
	}
    
	index = 0;
	for (MyFaceIter_HKS f = faces.begin(); f != faces.end(); f++) {
		f->index = index;
		index++;
	}
}

void MyMesh_HKS::checkIsolatedVertices ()
{
	for (MyVertexCIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		if (v->isIsolated()) {
			std::cerr << "Warning: vertex " << v->index
					  << " is isolated (not contained in any face)."
					  << std::endl;
		}
	}
}

void MyMesh_HKS::preallocateMeshElements (const MeshData_HKS& data)
{
	// count the number of edges
	std::set<std::pair<int,int>> _edges;
	for (std::vector<std::vector<Index_HKS>>::const_iterator f  = data.indices.begin();
														 f != data.indices.end();
														 f ++) {
		for (unsigned int I = 0; I < f->size(); I++) {
			int J = (I+1) % f->size();
			int i = (*f)[I].position;
			int j = (*f)[J].position;
            
			if (i > j) std::swap(i, j);
            
			_edges.insert(std::pair<int,int>(i, j));
		}
														 }
    
	size_t nV = data.positions.size();
	size_t nE = _edges.size();
	size_t nF = data.indices.size();
	size_t nHE = 2*nE;
	size_t chi = nV - nE + nF;
	int nB = std::max(0, 2 - (int)chi); // conservative approximation of number of boundary cycles
    
	halfEdges.clear();
	vertices.clear();
	edges.clear();
	faces.clear();
	boundaries.clear();
    
	halfEdges.reserve(nHE);
	vertices.reserve(nV);
	edges.reserve(nE);
	faces.reserve(nF + nB);
}

void MyMesh_HKS::normalize()
{
	// compute center of mass
	Eigen::Vector3d cm = Eigen::Vector3d::Zero();
	for (MyVertexCIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		cm += v->position;
	}
	cm /= (double)vertices.size();
    
	// translate to origin and determine radius
	double rMax = 0;
	for (MyVertexIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		v->position -= cm;
		rMax = std::max(rMax, v->position.norm());
	}
    
	// rescale to unit sphere
	for (MyVertexIter_HKS v = vertices.begin(); v != vertices.end(); v++) {
		v->position /= rMax;
	}
}
*/
