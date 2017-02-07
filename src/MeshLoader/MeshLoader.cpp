#include "ironhide-vision/MeshLoader.hpp"

namespace ih {

MeshLoader::MeshLoader() : m_mesh_filename(), m_mesh_path(), m_mesh_initialized(false) {

}
	
void MeshLoader::loadMesh(std::string t_mesh_filename, std::string t_mesh_path) {
	if(t_mesh_path.empty()) {
		throw std::invalid_argument("Mesh path parameter is empty");
	}

	if(t_mesh_filename.empty()) {
		throw std::invalid_argument("Mesh filename parameter is empty");
	}

	if(*t_mesh_path.rbegin() != '/') {
		t_mesh_path.push_back('/');
	}

  	if( t_mesh_filename.find_last_of(".") == std::string::npos || 
  		t_mesh_filename.substr(t_mesh_filename.find_last_of(".")) != ".stl") {
    	t_mesh_filename.append(".stl");
  	}

	this->m_mesh_filename = t_mesh_filename;
	this->m_mesh_path = t_mesh_path;

	std::cout << "Loads mesh: " << this->m_mesh_path << this->m_mesh_filename << std::endl;

	// Note: This will cause a segmentation fault if the file does not exist
	pcl::io::loadPolygonFileSTL(this->m_mesh_path + this->m_mesh_filename, this->m_mesh);

	m_mesh_initialized = true;
}

inline void randomPointTriangle(
        float ax,
        float ay,
        float az,
        float bx,
        float by,
        float bz,
        float cx,
        float cy,
        float cz,
        Eigen::Vector4f& p) {
    float r1 = static_cast<float> ((float)rand()/(RAND_MAX));
    float r2 = static_cast<float> ((float)rand()/(RAND_MAX));
    float r1sqr = sqrtf(r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    ax *= OneMinR1Sqr;
    ay *= OneMinR1Sqr;
    az *= OneMinR1Sqr;
    bx *= OneMinR2;
    by *= OneMinR2;
    bz *= OneMinR2;
    cx = r1sqr * (r2 * cx + bx) + ax;
    cy = r1sqr * (r2 * cy + by) + ay;
    cz = r1sqr * (r2 * cz + bz) + az;
    p[0] = cx;
    p[1] = cy;
    p[2] = cz;
    p[3] = 0;
}

//! Generates a point cloud from a mesh with random sampling
/*!
 * /param mesh The mesh to sample
 * /param num_samples The number of samples for the point cloud
 * /param cloud_out The output point cloud
 */
void generatePointCloud(
        const pcl::PolygonMesh mesh,
        const size_t num_samples,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
    // Create a VTK mesh
    vtkSmartPointer<vtkPolyData> meshVTK;
    pcl::VTKUtils::convertToVTK(mesh, meshVTK);
    meshVTK->BuildCells();
    vtkSmartPointer<vtkCellArray> cells = meshVTK->GetPolys();
    std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
    double pointA[3], pointB[3], pointC[3], totalArea = 0;
    size_t i = 0;
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    for(cells->InitTraversal();cells->GetNextCell(npts,ptIds);i++){
        meshVTK->GetPoint(ptIds[0], pointA);
        meshVTK->GetPoint(ptIds[1], pointB);
        meshVTK->GetPoint(ptIds[2], pointC);
        totalArea += vtkTriangle::TriangleArea(pointA, pointB, pointC);
        cumulativeAreas[i] = totalArea;
    }

    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    point_cloud.points.resize(num_samples);
    point_cloud.width = static_cast<pcl::uint32_t>(num_samples);
    point_cloud.height = 1;

    // Sample the mesh
    npts = 0; ptIds = NULL;
    for(i = 0; i < num_samples; i++) {
        float randomNumber = (float) i/ (float)num_samples * totalArea;
        std::vector<double>::iterator low = std::lower_bound(cumulativeAreas.begin(), cumulativeAreas.end(),
                                                             randomNumber);
        vtkIdType el = vtkIdType(low - cumulativeAreas.begin());
        Eigen::Vector4f point;
        meshVTK->GetCellPoints(el, npts, ptIds);
        meshVTK->GetPoint(ptIds[0], pointA);
        meshVTK->GetPoint(ptIds[1], pointB);
        meshVTK->GetPoint(ptIds[2], pointC);
        randomPointTriangle(
                float(pointA[0]), float(pointA[1]), float(pointA[2]),
                float(pointB[0]), float(pointB[1]), float(pointB[2]),
                float(pointC[0]), float(pointC[1]), float(pointC[2]), point);
        point_cloud.points[i].x = point[0];
        point_cloud.points[i].y = point[1];
        point_cloud.points[i].z = point[2];
        point_cloud.points[i].r = 255;
        point_cloud.points[i].g = 255;
        point_cloud.points[i].b = 255;
    }
    *cloud_out = point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MeshLoader::getMeshCloud() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	if(this->m_mesh_initialized) {

        vtkSmartPointer<vtkPolyData> meshVTK;
        pcl::VTKUtils::convertToVTK(this->m_mesh, meshVTK);

        pcl::visualization::PCLVisualizer generator("Generating traces...");
        generator.addModelFromPolyData (meshVTK, "mesh", 0);
        std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > clouds;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
        std::vector<float> enthropies;

        // Generate traces
        generator.renderViewTesselatedSphere(200, 200, clouds, poses, enthropies, 1);

        for(int i =0; i < 1; i++)
        {
            //Eigen::Matrix4f transform = poses.at(i).inverse();
            //pcl::transformPointCloud(clouds.at(i), clouds.at(i), transform);
            *point_cloud += clouds.at(i);
	   }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*point_cloud, *p_cloud);
/*
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(p_cloud);
    voxel_grid.setLeafSize(0.3f, 0.3f, 0.3f);
    voxel_grid.filter(*p_cloud);
*/
	return p_cloud;
}

}