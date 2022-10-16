#include "MyKeypointDetector_ISS.h"
// Fill out your copyright notice in the Description page of Project Settings.


// THIRD_PARTY_INCLUDES_START
//
// #pragma push_macro("check")
// #undef check
// #include <boost/intrusive/slist.hpp>
// #pragma pop_macro("check")
//
// THIRD_PARTY_INCLUDES_END


void AMyKeypointDetector_ISS::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBA>());;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
	// pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

	// Fill in the model cloud

	//double model_resolution;

	// Compute model_resolution

	/*pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * model_resolution);
	iss_detector.setNonMaxRadius(4 * model_resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(model);
	iss_detector.compute(*model_keypoints);*/
}

void AMyKeypointDetector_ISS::BeginPlay()
{
	Super::BeginPlay();
}

void AMyKeypointDetector_ISS::InitKeypointDetection()
{
	Super::InitKeypointDetection();
}

void AMyKeypointDetector_ISS::InitSelectedVertexLocation()
{
	Super::InitSelectedVertexLocation();
}

void AMyKeypointDetector_ISS::UpdateSelectedVertexLocation()
{
	Super::UpdateSelectedVertexLocation();
}
