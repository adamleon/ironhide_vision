#include "ray_trace_cloud_loader.h"

namespace ih
{

RayTraceCloudLoader::RayTraceCloudLoader(std::string mesh_path, std::string mesh_name, float scale) : RayTraceCloudLoader(mesh_name) {
    std::cout << "Loading STL file " << mesh_path.c_str() << mesh_name.c_str() << std::endl;
    pcl::io::loadPolygonFileSTL(mesh_path + mesh_name, this->mesh);
    pcl::PointCloud<pcl::PointXYZ> scaled_mesh_cloud;
    Eigen::Matrix4f scalefactor = Eigen::Matrix4f::Identity() * scale;
    pcl::fromPCLPointCloud2(mesh.cloud, scaled_mesh_cloud);
    pcl::transformPointCloud(scaled_mesh_cloud, scaled_mesh_cloud, scalefactor);
    pcl::toPCLPointCloud2(scaled_mesh_cloud, mesh.cloud);
}

RayTraceCloudLoader::RayTraceCloudLoader(pcl::PolygonMesh mesh, std::string mesh_name) : RayTraceCloudLoader(mesh_name) {
    this->mesh = mesh;
}

RayTraceCloudLoader::RayTraceCloudLoader(std::string mesh_name) {
    this->setTesselation_level(1);
    this->setCloudResolution(200);
    this->setPath("./trace_clouds/");
    this->mesh_name = mesh_name;
}

void RayTraceCloudLoader::populateLoader() {
    if(!this->loadPointClouds()) {
        if(this->mesh.cloud.data.size() == 0) {
            MeshLoadException e;
            throw e;
            return;
        }
        this->generatePointClouds();
        this->savePointClouds();
    }
}

std::vector<RayTraceCloud> RayTraceCloudLoader::getPointClouds(bool load){
    // Populate the loader if empty
    if(load && this->ray_trace_clouds.empty()) {
        this->populateLoader();
    }

    return this->ray_trace_clouds;
}

void RayTraceCloudLoader::generatePointClouds() {
    // Create mesh object
    vtkSmartPointer<vtkPolyData> meshVTK;
    pcl::VTKUtils::convertToVTK(this->mesh, meshVTK);

    // Set up trace generation
    std::cout << "Generating traces..." << std::endl;
    std::cout << "\033[32m  Current settings:" << std::endl;
    std::cout << "\033[32m    -mesh_name: " << this->mesh_name.c_str() << std::endl;
    std::cout << "\033[32m    -cloud_resolution: " << this->cloud_resolution << std::endl;
    std::cout << "\033[32m    -tesselation_level: " << this->tesselation_level << std::endl;

    pcl::visualization::PCLVisualizer generator("Generating traces...");
    generator.addModelFromPolyData (meshVTK, "mesh", 0);
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > clouds;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> enthropies;

    // Generate traces
    generator.renderViewTesselatedSphere(this->cloud_resolution, this->cloud_resolution, clouds, poses, enthropies, this->tesselation_level);

    // Generate clouds
    this->ray_trace_clouds.clear();
    for(int i =0; i < clouds.size(); i++)
    {
        RayTraceCloud cloud;
        cloud.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud.cloud = clouds.at(i);
      cloud.pose = poses.at(i);
        cloud.enthropy = enthropies.at(i);
        this->ray_trace_clouds.push_back(cloud);
    }
}

bool RayTraceCloudLoader::savePointClouds() {
    if(this->ray_trace_clouds.empty()){
        return false;
    }

    // Set saving path
    std::string save_path = this->path + "/" + this->mesh_name + "/";
    std::cout << "Saving ray traces" << std::endl;
    std::cout << "\tUsing " << save_path.c_str() << std::endl;

    // Generate YAML node
    YAML::Node clouds;
    for(int i = 0; i < this->ray_trace_clouds.size(); i++) {
        RayTraceCloud ray_trace = this->ray_trace_clouds.at(i);

        std::stringstream filename;
        filename << this->mesh_name << "_cloud_";
        filename << setfill('0') << setw(4) << (i+1);
        filename << ".pcd";
        boost::filesystem::create_directories(save_path);
        pcl::io::savePCDFile(save_path + filename.str(), *ray_trace.cloud);

        YAML::Node node;
        node["cloud"] = filename.str();
        for(int j = 0; j < 16; ++j) {
            node["pose"].push_back(ray_trace.pose(j / 4, j % 4));
        }
        node["enthropy"] = ray_trace.enthropy;

        std::stringstream cloud_node;
        cloud_node << setfill('0') << setw(4) << (i+1);
        clouds[cloud_node.str()] = node;
    }

    // Saving the YAML node
    YAML::Emitter out;
    out << clouds;
    boost::filesystem::ofstream f(save_path + this->mesh_name + ".yaml");
    f << out.c_str();
    std::cout << "\033[33mSuccessfully saved " << (int)this->ray_trace_clouds.size() << " ray traces" << std::endl;
};


bool RayTraceCloudLoader::loadPointClouds() {
    // Set the save path
    std::string save_path = this->path + "/" + this->mesh_name + "/";
    YAML::Node clouds;

    // Try to load the files
    try {
        std::cout << "Loading ray trace clouds..." << std::endl;
        std::cout << "\tUsing " << save_path.c_str() << std::endl;
        clouds = YAML::LoadFile(save_path + this->mesh_name + ".yaml");
    }
    catch (const std::exception& e) {
        std::cout << "\033[31m" << e.what() << "\033[""0m" << std::endl;
        return  false;
    }

    int i = 1;
    std::stringstream cloud_node;
    cloud_node << setfill('0') << setw(4) << i;
    this->ray_trace_clouds.clear();

    // Populate ray_trace_cloud from the YAML node
    while(clouds[cloud_node.str()]){
        YAML::Node cloud = clouds[cloud_node.str()];
        RayTraceCloud ray_trace;
        ray_trace.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(save_path + cloud["cloud"].as<std::string>(), *ray_trace.cloud);
        for(int x = 0; x < 4; ++x) {
            for(int y = 0; y < 4; ++y) {
                ray_trace.pose(x, y) = cloud["pose"][(int)(x*4 + y)].as<float>();
            }
        }
        ray_trace.enthropy = cloud["enthropy"].as<float>();
        this->ray_trace_clouds.push_back(ray_trace);
        cloud_node.clear();
        cloud_node.str(std::string());
        cloud_node << setfill('0') << setw(4) << ++i;
    }
    std::cout << "\033[33mSuccessfully loaded " << i-1 << " ray traces\033[0m" << std::endl;

    return true;
}

} // End of namespace