#ifndef RAY_TRACE_CLOUD_LOADER_H
#define RAY_TRACE_CLOUD_LOADER_H

#include <pcl/common/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <string.h>
#include <sstream>
#include <iomanip>

#include "ray_trace_cloud.h"
#include "ray_trace_cloud_exceptions.h"

namespace ih
{

class RayTraceCloudLoader {
public:
    //! Initializes the variables for the loader. This loads the STL file and generates point clouds from it.
    /*!
     * /param mesh_path The path of the STL file
     * /param mesh_name The name of the mesh. Used for saving and loading the files.
     * /param scale The scale of the model. This library measures in meters, so scale accordingly.
     */
    RayTraceCloudLoader(std::string mesh_path, std::string mesh_name, float scale=0.001f);

    //! Initializes the variables for the loader
    /*!
     * /param mesh The mesh that will be traced,
     * /param mesh_name The name of the mesh. Used for saving and loading the files.
     */
    RayTraceCloudLoader(pcl::PolygonMesh mesh, std::string mesh_name);

    //! Initializes the variables for the loader. This only works if the traces already exist
    /*!
     * /param mesh_name The name of the mesh. Used for saving and loading the files.
     */
    RayTraceCloudLoader(std::string mesh_name);

    //! Gets the loaded ray trace clouds
    /*!
     * /param load If true, the function will call populateLoader if there are no ray traces.
     */
    std::vector<RayTraceCloud> getPointClouds(bool load = false);

    //! Populates the loader with point clouds
    /*!
     * This will either load existing saved traces, or generate and save traces if they do not exist.
     */
    void populateLoader();

    //! Sets the tesselation level for generation
    /*!
     * /param tesselation_level The level of tesselation. Default is 1 (creating 42 images)
     */
    void setTesselation_level(int tesselation_level) {
        RayTraceCloudLoader::tesselation_level = tesselation_level;
    }

    //! Sets the cloud resolution for generation
    /*!
     * /param cloud_resolution The resolution of the generated image. Default is 200
     */
    void setCloudResolution(int cloud_resolution) {
        RayTraceCloudLoader::cloud_resolution = cloud_resolution;
    }

    //! Sets the path for saving and loading of files
    /*!
     * /param path The path to search for the trace files. Default is ironhide_vision_control/trace_clouds/
     */
    void setPath(const std::string &path) {
        RayTraceCloudLoader::path = path;
    }

private:
    //! Generate the point clouds from a mesh
    /*!
     * This function will generate the traces from a mesh and populate the ray_trace_clouds variable
     */
    void generatePointClouds();

    //! Loads the ray trace point clouds from files
    /*!
     * This function will load and populate the ray_trace_clouds variable from the given path
     */
    bool loadPointClouds();

    //! Saves the ray trace point clouds to files
    /*!
     * This function will save all teh infromation from the ray_trace_clouds variable to the given path
     */
    bool savePointClouds();

    std::string path; //!< The path for saving and loading files.
    std::string mesh_name; //!< The name of the mesh. Used for saving and loading file names.
    std::vector<RayTraceCloud> ray_trace_clouds; //!< List of ray trace clouds.
    pcl::PolygonMesh mesh; //!< The mesh which is used for generation
    int cloud_resolution; //!< The resolution camera when generating clouds
    int tesselation_level; //!< The tesselation level of the sphere for the camera
};

} // End of namespace

#endif // RAY_TRACE_CLOUD_LOADER_H
