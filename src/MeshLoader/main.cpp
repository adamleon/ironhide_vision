#include "ironhide-vision/MeshLoader.hpp"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char* argv[]) {
	std::cout << "Hello World" << std::endl;

	ih::MeshLoader ml;

	std::cout << "Hello World" << std::endl;
	ml.getMeshCloud();
	ml.loadMesh("FreakThing", "/home/adamleon/workspaces/ironhide_ws/src/ironhide_vision/test/stl-files/");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = ml.getMeshCloud();

	Eigen::Matrix4f scalefactor = Eigen::Matrix4f::Identity() * 0.001f;

    pcl::transformPointCloud(*cloud, *cloud, scalefactor);

	int length = cloud->size();


	std::cout << "Hello World" << length << std::endl;

	/*

	Eigen::Vector3d vectors[length];
	for(int i = 0; i < length; ++i) {
		 Eigen::Vector3d v(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		 vectors[i] = v;
	}

	std::cout << "Hello World" << length << std::endl;


	std::cout << "Hello World2" << std::endl;
	for(int i = 0; i < length; ++i) {
		Eigen::Vector3d vec = vectors[i];
		Eigen::Matrix3d covariance;
		int size = 0;


		for(int j = 0; j < length; ++j) {
			if(i == j) continue;

			Eigen::Vector3d diff = (vec-vectors[j]);
			float dist = diff.norm();
			if(dist < 0.007f) {
				if(size == 0) covariance = diff*diff.transpose();
				else covariance += diff*diff.transpose();
				++size;
			}
		}
		//std::cout << size << std::endl;
		covariance = covariance/(float)size;

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
   		Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
   		float sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
   		float c_linear = (eigenvalues[2] - eigenvalues[1])/sum;
   		float c_planar = 2*(eigenvalues[1]-eigenvalues[0])/sum;
   		float c_spherical = 3*eigenvalues[0]/sum;

   		cloud->points[i].r = c_linear*255;
   		cloud->points[i].g = c_planar*255;
   		cloud->points[i].b = c_spherical*255;

   		if(i % (length/10) == 0) {
   			std::cout << (float)i/(float)length*100 << "%% finished" << std::endl;
   		}
	}
	*/

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.setBackgroundColor (255, 255, 255);
    viewer.addPointCloud(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.addCoordinateSystem (0.001f);
	viewer.initCameraParameters ();
    
    while (!viewer.wasStopped ())
    {
    	viewer.spinOnce();
    }

	return 0;
}