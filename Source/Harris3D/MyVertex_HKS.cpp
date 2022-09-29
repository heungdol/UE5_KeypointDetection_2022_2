#include "MyVertex_HKS.h"

#include <ThirdParty/Imath/Deploy/Imath-3.1.3/include/Imath/ImathPlatform.h>

#include "MyHalfEdge_HKS.h"
#include "MyFace_HKS.h"

std::vector<MyHalfEdge_HKS> isolated;

bool MyVertex_HKS::isIsolated() const
{
    return he == isolated.begin();
}

bool MyVertex_HKS::isBoundary() const
{
    MyHalfEdgeCIter_HKS h = he;
    do {
        if (h->onBoundary) return true;
        
        h = h->flip->next;
    } while (h != he);
    
    return false;
}

int MyVertex_HKS::degree() const
{
    int k = 0;
    MyHalfEdgeCIter_HKS h = he;
    do {
        k++;
        
        h = h->flip->next;
    } while (h != he);
    
    return k;
}

Eigen::Vector3d MyVertex_HKS::normal() const
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    if (isIsolated()) return normal;
    
    MyHalfEdgeCIter_HKS h = he;
    do {
        Eigen::Vector3d e1 = h->next->vertex->position - position;
        Eigen::Vector3d e2 = h->next->next->vertex->position - position;
        
        double d = e1.dot(e2) / sqrt(e1.squaredNorm() * e2.squaredNorm());
        if (d < -1.0) d = -1.0;
        else if (d >  1.0) d = 1.0;
        double angle = acos(d);
        
        Eigen::Vector3d n = h->face->normal;
        normal += angle * n;
        
        h = h->flip->next;
    } while (h != he);
    
    if (!normal.isZero()) normal.normalize();
    return normal;
}

double MyVertex_HKS::dualArea() const
{
    double area = 0.0;
    
    MyHalfEdgeCIter_HKS h = he;
    do {
        area += h->face->area();
        h = h->flip->next;
        
    } while (h != he);
    
    return area / 3.0;
}

double MyVertex_HKS::angleDefect() const
{
    double defect = 2 * M_PI;
    
    MyHalfEdgeCIter_HKS h = he;
    do {
        Eigen::Vector3d u = h->next->vertex->position - h->vertex->position;
        Eigen::Vector3d v = h->next->next->vertex->position - h->vertex->position;
        
        double theta = acos(u.dot(v) / (u.norm() * v.norm()));
        defect -= theta;
        
        h = h->flip->next;
        
    } while (h != he);
    
    return defect;
}

bool MyVertex_HKS::isFeature(int t, int N, int depth) const
{
    std::queue<const MyVertex_HKS *> queue;
    std::unordered_map<int, bool> visited;
    
    // enqueue
    queue.push(this);
    queue.push(NULL);
    visited[index] = true;
    int levels = 0;
    
    // perform bfs
    while (!queue.empty()) {
        const MyVertex_HKS *v = queue.front();
        queue.pop();
        
        if (v == NULL) {
            levels++;
            queue.push(NULL);
            if (queue.front() == NULL || levels == depth) break;
            
        } else {
            MyHalfEdgeCIter_HKS h = v->he;
            do {
                const MyVertex_HKS *vn = &(*h->flip->vertex);
                if (!visited[vn->index]) {
                    // check if descriptor value for a particular t is greater than the neighbor's value
                    // TODO

                    if (t < 0 || t >= N)
                        return false;
                    
                    if (descriptor (t) < vn->descriptor(t)) return false;
                    
                    queue.push(vn);
                    visited[vn->index] = true;
                }
                
                h = h->flip->next;
            } while (h != v->he);
        }
    }
    
    return true;
}
