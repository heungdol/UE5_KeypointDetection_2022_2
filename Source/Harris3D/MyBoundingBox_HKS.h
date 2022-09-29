#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "MyType_HKS.h"

class MyBoundingBox_HKS {
public:
    // constructor
    MyBoundingBox_HKS();
    
    // initialize with specified components
    MyBoundingBox_HKS(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0);
    
    // initialize with specified components
    MyBoundingBox_HKS(const Eigen::Vector3d& p);
    
    // expand bounding box to include point/ bbox
    void expandToInclude(const Eigen::Vector3d& p);
    void expandToInclude(const MyBoundingBox_HKS& b);

    // return the max dimension
    int maxDimension() const;
    
    // check if bounding box and face intersect
    bool intersect(const Eigen::Vector3d& p, double& dist) const;
    
    // member variables
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    Eigen::Vector3d extent;
};

#endif
