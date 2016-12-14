#include "ironhide-vision/MeshLoader.hpp"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char* argv[]) {
	std::cout << "Hello World" << std::endl;

	ih::MeshLoader ml;

	std::cout << "Hello World" << std::endl;
	ml.getMeshCloud();
	ml.loadMesh("FreakThing", "/home/adamleon/workspaces/ironhide_ws/src/ironhide_vision/test/stl-files/");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = ml.getMeshCloud();

	int length = cloud->size();
	Eigen::Vector3d vectors[length];
	for(int i = 0; i < length; ++i) {
		 Eigen::Vector3d v(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		 vectors[i] = v;
	}

	std::cout << "Hello World2" << std::endl;
	
	Eigen::Matrix3d covariance[length];

	std::cout << "Hello World2" << std::endl;
	for(int i = 0; i < length; ++i) {
		Eigen::Vector3d vec = vectors[i];
		int size = 0;


		for(int j = 0; j < length; ++j) {
			if(i == j) continue;

			Eigen::Vector3d diff = (vec-vectors[j]);
			float dist = diff.norm();
			if(dist < 10.0f) {
				if(size == 0) covariance[i] = diff*diff.transpose();
				else covariance[i] += diff*diff.transpose();
				++size;
			}
		}
		covariance[i] = covariance[i]/(float)size;
	}


	for(int i = 0; i < length; ++i) {
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance[i]);
   		std::cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << std::endl;
	}


	std::cout << covariance[0] << std::endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    while (!viewer.wasStopped ())
    {
    
    }

	return 0;
}