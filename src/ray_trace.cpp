//
// Created by adamleon on 01.02.16.
//

#include "hahaha/ray_trace.hpp"
#include "../../../../../../../../usr/include/pcl-1.7/pcl/impl/point_types.hpp"



RayTraceLoader::RayTraceLoader(pcl::PolygonMesh mesh, std::string mesh_name) : RayTraceLoader(mesh_name) {
    this->mesh = mesh;
}

RayTraceLoader::RayTraceLoader(std::string mesh_name) {
    this->setTesselation_level(1);
    this->setCloudResolution(960);
    this->setPath(ros::package::getPath("hahaha") + "/trace_clouds/");
    this->mesh_name = mesh_name;
}

void RayTraceLoader::populateLoader() {
    if(!this->loadPointClouds()) {
        if(this->mesh.cloud.data.size() == 0) {
            ROS_ERROR("There is no defined mesh to generate clouds from");
            return;
        }
        this->generatePointClouds();
        this->savePointClouds();
    }
}

std::vector<RayTraceCloud> RayTraceLoader::getPointClouds(bool load){
    // Populate the loader if empty
    if(load && this->ray_trace_clouds.empty()) {
        this->populateLoader();
    }

    return this->ray_trace_clouds;
}

void RayTraceLoader::generatePointClouds() {
    // Create mesh object
    vtkSmartPointer<vtkPolyData> meshVTK;
    pcl::VTKUtils::convertToVTK(this->mesh, meshVTK);

    // Set up trace generation
    ROS_INFO("Generating traces...");
    ROS_INFO("\033[32m  Current settings:");
    ROS_INFO("\033[32m    -mesh_name: %s", this->mesh_name.c_str());
    ROS_INFO("\033[32m    -cloud_resolution: %d", this->cloud_resolution);
    ROS_INFO("\033[32m    -tesselation_level: %d", this->tesselation_level);

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

bool RayTraceLoader::savePointClouds() {
    if(this->ray_trace_clouds.empty()){
        return false;
    }

    // Set saving path
    std::string save_path = this->path + "/" + this->mesh_name + "/";
    ROS_INFO("Saving ray traces");
    ROS_INFO("\tUsing %s", save_path.c_str());

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
        for(int j = 0; j < 16; j++) {
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
    ROS_INFO("\033[33mSuccessfully saved %d ray traces", (int)this->ray_trace_clouds.size());
};


bool RayTraceLoader::loadPointClouds() {
    // Set the save path
    std::string save_path = this->path + "/" + this->mesh_name + "/";
    YAML::Node clouds;

    // Try to load the files
    try {
        ROS_INFO("Loading ray trace clouds...");
        ROS_INFO("\tUsing %s", save_path.c_str());
        clouds = YAML::LoadFile(save_path + this->mesh_name + ".yaml");
    }
    catch (const std::exception& e) {
        ROS_INFO("\033[31mFile not found.\033[0m");
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
        for(int x = 0; x < 4; x++) {
            for(int y = 0; y < 4; y++) {
                ray_trace.pose(x, y) = cloud["pose"][(int)(x*4 + y)].as<float>();
            }
        }
        ray_trace.enthropy = cloud["enthropy"].as<float>();
        this->ray_trace_clouds.push_back(ray_trace);
        cloud_node.clear();
        cloud_node.str(std::string());
        cloud_node << setfill('0') << setw(4) << ++i;
    }
    ROS_INFO("\033[33mSuccessfully loaded %d ray traces\033[0m", i-1);
    return true;
}

