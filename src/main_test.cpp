#include <iostream>
#include <Eigen/Core>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vsr/space/vsr_cga3D_op.h>
#include <vsr/space/vsr_cga3D_round.h>
#include <stdio.h>
#include <math.h>

double mexicanhat(double sdistance) {
   double sigma = 1;
   return 2/(std::sqrt(3*sigma*std::sqrt(3.1415)))*(1-((sdistance)/(sigma*sigma))*std::exp(-(sdistance)/(2*sigma*sigma)));
}

int main(int argc, char* argv[]) {
	std::cout << "Hello World" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

   if(pcl::io::loadPCDFile("/home/adamleon/workspaces/ironhide_ws/src/ironhide_vision/THINGY2_cloud_0001.pcd", *cloud) == -1)
      std::cout << "This went wrong" << std::endl;
   
   Eigen::Matrix4d transform;
  // transform << 0, -0.8506508, -0.5257311, -0.07423134,
   //             1, 0, 0, 0.0005360948,
   //             0, -0.5257311, 0.8506508, 0.8886045,
   //             0, 0, 0, 1;

   transform << 0, 1, 0, -0.0005,
                -0.8506508, 0, -0.5257311, 0.4040,
               -0.5257311, 0, 0.8506508, -0.6538,
               0, 0, 0, 1;


   pcl::transformPointCloud (*cloud, *cloud, transform);

   vsr::cga::DualSphere sphere = vsr::cga::Construct::sphere(0,0,0,1);
   int size = cloud->points.size();
   vsr::cga::Pnt cga_points[size];
   for(int i = 0; i < size; ++i){
      vsr::cga::Point points = vsr::cga::Vec(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z).null();
      vsr::cga::Line line = vsr::cga::Ori(1) ^ points ^ vsr::cga::Inf(1);
      vsr::cga::Pair ppair = (sphere<=line);
      auto pp = vsr::cga::Construct::pointA(ppair);
      auto pn = vsr::cga::Construct::pointB(ppair);
      if(vsr::cga::Round::squaredDistance(pp,points) < vsr::cga::Round::squaredDistance(pn,points)){
         cga_points[i] = pp;
      }
      else{
         cga_points[i] = pn;
      }
   }

   double neighbours[size];
   double largest = 0;
   for(int i = 0; i < size; ++i) {
      neighbours[i] = 0;
      for(int j = 0; j < size; ++j) {
         if(i == j) continue;

         if(vsr::cga::Round::distance(cga_points[i],cga_points[j]) < 0.5) {
            neighbours[i] += mexicanhat(vsr::cga::Round::squaredDistance(cga_points[i],cga_points[j]));
         }
      }
      if(neighbours[i] > largest) {
         largest = neighbours[i];
      }
   }

   for(int i = 0; i < size; ++i){
      pcl::PointXYZRGB p;
      p.x = cga_points[i][0];
      p.y = cga_points[i][1];
      p.z = cga_points[i][2];
      p.g = ((float)neighbours[i])/((float)largest)*255;
      p.b = ((float)neighbours[i])/((float)largest)*255;
      p.r = 255;
      result->points.push_back(p);
   }

   	//... populate cloud
   pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
   
   float min = 1000;
   float max = -1000;
   for(int i = 0; i < cloud->points.size(); ++i){
      float x = cloud->points[i].z;
      if(x < min) min = x;
      if(x > max) max = x; 
   }

   std::cout << min << ", " << max << std::endl;

   //blocks until the cloud is actually rendered
   viewer.setBackgroundColor (0, 0, 0);
   viewer.addPointCloud(result, "sample cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer.addCoordinateSystem (0.1f);
   viewer.initCameraParameters ();
    
   while (!viewer.wasStopped ())
   {
      viewer.spinOnce();
   }

	return 0;
}


 