#include "MyMeshIO_HKS.h"
#include "MyType_HKS.h"
#include <set>
#include <map>
#include <iomanip>

#include "KismetProceduralMeshLibrary.h"
#include "MyMesh_HKS.h"

Index_HKS parseFaceIndex(const std::string& token)
{
    std::stringstream in(token);
    std::string indexString;
    int indices[3] = {-1, -1, -1};
    
    int i = 0;
    while(getline(in,indexString,'/')) {
        if (indexString != "\\") {
            std::stringstream ss(indexString);
            ss >> indices[i++];
        }
    }
    
    // decrement since indices in OBJ files are 1-based
    return Index_HKS(indices[0]-1,
                 indices[1]-1,  
                 indices[2]-1);
}

std::string stringRep(const Eigen::Vector3d& v)
{
    return std::to_string(v.x()) + " " + std::to_string(v.y()) + " " + std::to_string(v.z());
}

void MyMeshIO_HKS::preallocateMeshElements(const MeshData_HKS& data, MyMesh_HKS& mesh)
{
    // count the number of edges
    std::set<std::pair<int,int>> edges;
    for (std::vector<std::vector<Index_HKS>>::const_iterator f  = data.indices.begin();
                                                         f != data.indices.end();
                                                         f ++) {
        for (unsigned int I = 0; I < f->size(); I++) {
            int J = (I+1) % f->size();
            int i = (*f)[I].position;
            int j = (*f)[J].position;
            
            if (i > j) std::swap(i, j);
            
            edges.insert(std::pair<int,int>(i, j));
        }
    }
    
    size_t nV = data.positions.size();
    size_t nE = edges.size();
    size_t nF = data.indices.size();
    size_t nHE = 2*nE;
    size_t chi = nV - nE + nF;
    int nB = std::max(0, 2 - (int)chi); // conservative approximation of number of boundary cycles
    
    mesh.halfEdges.clear();
    mesh.vertices.clear();
    mesh.edges.clear();
    mesh.faces.clear();
    mesh.boundaries.clear();
    
    mesh.halfEdges.reserve(nHE);
    mesh.vertices.reserve(nV);
    mesh.edges.reserve(nE);
    mesh.faces.reserve(nF + nB);
}

void MyMeshIO_HKS::indexElements(MyMesh_HKS& mesh)
{
    int index = 0;
    for (MyVertexIter_HKS v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->index = index;
        index++;
    }
    
    index = 0;
    for (MyEdgeIter_HKS e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        e->index = index;
        index++;
    }
    
    index = 0;
    for (MyHalfEdgeIter_HKS h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
        h->index = index;
        index++;
    }
    
    index = 0;
    for (MyFaceIter_HKS f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        f->index = index;
        index++;
    }
}

void MyMeshIO_HKS::checkIsolatedVertices(const MyMesh_HKS& mesh)
{
    for (MyVertexCIter_HKS v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (v->isIsolated()) {
            std::cerr << "Warning: vertex " << v->index
                      << " is isolated (not contained in any face)."
                      << std::endl;
        }
    }
}

void MyMeshIO_HKS::checkNonManifoldVertices(const MyMesh_HKS& mesh)
{
    std::unordered_map<std::string, int> vertexFaceMap;
    
    for (MyFaceCIter_HKS f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        MyHalfEdgeCIter_HKS he = f->he;
        do {
            vertexFaceMap[stringRep(he->vertex->position)] ++;
            he = he->next;
            
        } while (he != f->he);
    }
    
    for (MyVertexCIter_HKS v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
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

extern std::vector<MyHalfEdge_HKS> isolated;

bool MyMeshIO_HKS::read(const UStaticMeshComponent* sm, MyMesh_HKS& mesh)
{
    if (sm == NULL)
    {
        mesh.enableModel = false;
        return false;
    }

    MeshData_HKS data;
	
    int numVertices, numFaces, numNormals, numUV;
	
    // Static Mesh 정보 가져오기
    TArray <FVector> verts;
    TArray <int> tris;
    TArray <FVector> nors;
    TArray <FVector2D> uvs;
    TArray<FProcMeshTangent> tans;

    UKismetProceduralMeshLibrary::GetSectionFromStaticMesh(sm->GetStaticMesh(), 0, 0, verts, tris, nors, uvs, tans);
	
    numVertices = verts.Num();
    numNormals = nors.Num();
    numFaces = tris.Num ();
    numUV = uvs.Num();

    if (numVertices <= 0)
    {
        mesh.enableModel = false;
        return false;
    }

    // 버텍스 개수가 너무 많으면
    if (numVertices > VERTEX_NUMBER_MAX)
    {
        mesh.enableModel = false;
        return false;
    }

    // 버텍스
    for(int i = 0; i < numVertices; i++)
    {
        double xc, yc, zc;

        xc = verts [i].X;
        yc = verts [i].Y;
        zc = verts [i].Z;

        data.positions.push_back(Eigen::Vector3d(xc, yc, zc));
    }

    // UV
    for(int i = 0; i < numUV; i++)
    {
        double xc, yc;

        xc = uvs [i].X;
        yc = uvs [i].Y;

        data.uvs.push_back(Eigen::Vector3d(xc,yc,0));
    }

    // 노말
    for(int i = 0; i < numNormals; i++)
    {
        double xc, yc, zc;

        xc = nors [i].X;
        yc = nors [i].Y;
        zc = nors [i].Z;

        data.normals.push_back(Eigen::Vector3d(xc, yc, zc));
    }

    // UStaticMesh->Face Vertex 3개 묶음
    for(int i = 0, fi = 0;  i < numFaces; i+=3, fi++)
    {
        std::vector<Index_HKS> faceIndices;

        faceIndices.push_back(Index_HKS (tris[i], tris[i], tris[i]));
        faceIndices.push_back(Index_HKS (tris[i+1], tris[i+1], tris[i+1]));
        faceIndices.push_back(Index_HKS (tris[i+2], tris[i+2], tris[i+2]));

        data.indices.push_back(faceIndices);
    }
    
    return buildMesh(data, mesh);
}

bool MyMeshIO_HKS::buildMesh(const MeshData_HKS& data, MyMesh_HKS& mesh)
{
    std::map<std::pair<int, int>, int> edgeCount;
    std::map<std::pair<int, int>, MyHalfEdgeIter_HKS> existingHalfEdges;
    std::map<int, MyVertexIter_HKS> indexToVertex;
    std::map<MyHalfEdgeIter_HKS, bool> hasFlipEdge;
    
    preallocateMeshElements(data, mesh);
    
    // insert vertices into mesh and map vertex indices to vertex pointers
    for (unsigned int i = 0; i < data.positions.size(); i++) {
        MyVertexIter_HKS vertex = mesh.vertices.insert(mesh.vertices.end(), MyVertex_HKS());
        vertex->position = data.positions[i];
        vertex->he = isolated.begin();
        indexToVertex[i] = vertex;
    }
   
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
        MyFaceIter_HKS newFace = mesh.faces.insert(mesh.faces.end(), MyFace_HKS());
        
        // create a halfedge for each edge of the face
        std::vector<MyHalfEdgeIter_HKS> halfEdges(n);
        for (int i = 0; i < n; i++) {
            halfEdges[i] = mesh.halfEdges.insert(mesh.halfEdges.end(), MyHalfEdge_HKS());
        }
        
        // initialize the halfedges
        for (int i = 0; i < n; i++) {
            // vertex indices
            int a = (*f)[i].position;
            int b = (*f)[(i+1)%n].position;
            
            // set halfedge attributes
            halfEdges[i]->next = halfEdges[(i+1)%n];
            halfEdges[i]->vertex = indexToVertex[a];
            
            halfEdges[i]->onBoundary = false;
            
            // keep track of which halfedges have flip edges defined (for deteting boundaries)
            hasFlipEdge[halfEdges[i]] = false;
            
            // point vertex a at the current halfedge
            indexToVertex[a]->he = halfEdges[i];
            
            // point new face and halfedge to each other
            halfEdges[i]->face = newFace;
            newFace->he = halfEdges[i];
            
            // if an edge between a and b has been created in the past, it is the flip edge of the current halfedge
            if (a > b) std::swap(a, b);
            if (existingHalfEdges.find(std::pair<int, int>(a, b)) != existingHalfEdges.end()) {
                halfEdges[i]->flip = existingHalfEdges[std::pair<int, int>(a, b)];
                halfEdges[i]->flip->flip = halfEdges[i];
                halfEdges[i]->edge = halfEdges[i]->flip->edge;
                hasFlipEdge[halfEdges[i]] = true;
                hasFlipEdge[halfEdges[i]->flip] = true;
                
            } else {
                // create an edge and set its halfedge
                halfEdges[i]->edge = mesh.edges.insert(mesh.edges.end(), MyEdge_HKS());
                halfEdges[i]->edge->he = halfEdges[i];
                edgeCount[std::pair<int, int>(a, b)] = 0;
            }
            
            // record that halfedge has been created from a to b
            existingHalfEdges[std::pair<int, int>(a, b)] = halfEdges[i];
            
            // check for nonmanifold edges
            edgeCount[std::pair<int, int>(a, b)] ++;
            if (edgeCount[std::pair<int, int>(a, b)] > 2) {
                std::cerr << "Error: edge " << a << ", " << b << " is non manifold" << std::endl;
                return false;
            }
            else
            {
                // 노멀 구하기
                newFace->normal += data.normals [(*f)[i].position];
            }
            
        }
        
        faceIndex++;

        // 노멀 구하기
        newFace->normal = newFace->normal.normalized();
    }
    
    if (degenerateFaces) {
        return false;
    }
    
    // insert extra faces for boundary cycle
    for (MyHalfEdgeIter_HKS currHe = mesh.halfEdges.begin(); currHe != mesh.halfEdges.end(); currHe++) {
        // if a halfedge with no flip edge is found, create a new face and link it the corresponding boundary cycle
        if (!hasFlipEdge[currHe]) {
            // create face
            MyFaceIter_HKS newFace = mesh.faces.insert(mesh.faces.end(), MyFace_HKS());
            
            // walk along boundary cycle
            std::vector<MyHalfEdgeIter_HKS> boundaryCycle;
            MyHalfEdgeIter_HKS he = currHe;
            do {
                // create a new halfedge on the boundary face
                MyHalfEdgeIter_HKS newHe = mesh.halfEdges.insert(mesh.halfEdges.end(), MyHalfEdge_HKS());
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
                newFace->isBoundary = newHe->onBoundary;
                
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
            mesh.boundaries.insert(mesh.boundaries.end(), boundaryCycle[0]);
        }
    }
    
    indexElements(mesh);
    checkIsolatedVertices(mesh);
    checkNonManifoldVertices(mesh);
    
    return true;
}
