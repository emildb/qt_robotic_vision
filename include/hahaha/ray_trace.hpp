//
// Created by adamleon on 29.01.16.
// Modified by Emil Bjørlykhaug
//

#ifndef IRONHIDE_VISION_CONTROL_POINT_CLOUD_RAY_TRACE_HPP
#define IRONHIDE_VISION_CONTROL_POINT_CLOUD_RAY_TRACE_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <string.h>
#include <sstream>
#include <iomanip>

/*!
 * A structure containing all information about a ray trace
 */
struct RayTraceCloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; /*!< The point cloud of the ray trace */
    Eigen::Matrix4f pose; /*!< The pose transformation from the camera to the mesh when the ray trace was generated */
    float enthropy; /*!< The amount of the whole mesh seen in the camera */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor; /*!< VFH descriptor */
    pcl::PointCloud<pcl::Normal>::Ptr normals; /*!< Normals of the points in the Point Cloud */

};

//! A manager for the ray trace clouds
/*!
 * This manager loads up the ray traces of a specific mesh, or generates them from the mesh if they do not exist.
 */
class RayTraceLoader {
public:
    //! Initializes the variables for the loader
    /*!
     * /param mesh The mesh that will be traced,
     * /param mesh_name The name of the mesh. Used for saving and loading the files.
     */
    RayTraceLoader(pcl::PolygonMesh mesh, std::string mesh_name);


    //! Initializes the variables for the loader. This only works if the traces already exist
    /*!
     * /param mesh_name The name of the mesh. Used for saving and loading the files.
     */
    RayTraceLoader(std::string mesh_name);


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
        RayTraceLoader::tesselation_level = tesselation_level;
    }


    //! Sets the cloud resolution for generation
    /*!
     * /param cloud_resolution The resolution of the generated image. Default is 960
     */
    void setCloudResolution(int cloud_resolution) {
        RayTraceLoader::cloud_resolution = cloud_resolution;
    }


    //! Sets the path for saving and loading of files
    /*!
     * /param path The path to search for the trace files. Default is ironhide_vision_control/trace_clouds/
     */
    void setPath(const std::string &path) {
        RayTraceLoader::path = path;
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
#endif //IRONHIDE_VISION_CONTROL_POINT_CLOUD_RAY_TRACE_HPP
