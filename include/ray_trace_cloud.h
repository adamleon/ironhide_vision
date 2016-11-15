#ifndef RAY_TRACE_CLOUD_H
#define RAY_TRACE_CLOUD_H

#include <pcl/point_cloud.h>
#include <Eigen/Core>

/*!
 * A structure containing all information about a ray trace
 */
struct RayTraceCloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; /*!< The point cloud of the ray trace */
    Eigen::Matrix4f pose; /*!< The pose transformation from the camera to the mesh when the ray trace was generated */
    float enthropy; /*!< The amount of the whole mesh seen in the camera */
};

#endif // RAY_TRACE_CLOUD_H
