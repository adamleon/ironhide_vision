#ifndef MESH_LOADER_HPP
#define MESH_LOADER_HPP

#include <iostream>
#include <exception>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkTriangle.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Core>

namespace ih {

class MeshLoader {
public:
	MeshLoader();
	
	void loadMesh(std::string t_mesh_name, std::string t_mesh_path);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMeshCloud();

private:
	std::string m_mesh_filename;
	std::string m_mesh_path;
	pcl::PolygonMesh m_mesh;
	bool m_mesh_initialized;
};

}

#endif //MESH_LOADER_HPP