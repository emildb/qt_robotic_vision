//
// Created by Emil Bjørlykhaug 06.04.16
//

#ifndef hahaha_COMPUTING_H
#define hahaha_COMPUTING_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "pcl/visualization/pcl_visualizer.h"
#include "QVTKWidget.h"
#include "vtkRenderWindow.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transforms.h>
#include <vtkTriangle.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include "ray_trace.hpp"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/statistical_outlier_removal.h>


class Computing {
public:
    Computing();
    ~Computing();

    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> calculateVFHDescriptors(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr, float);
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr generate_search_tree(std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr>);
    std::vector<float> match_cloud(pcl::PointCloud<pcl::VFHSignature308>::Ptr, pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr);
    int search_for_correct_cluster(std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cluster_descriptors,std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cad_descriptors);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint, float radius);
    pcl::PointCloud<pcl::PointXYZ>::Ptr generate_keypoints_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr, float min_scale, int nr_octaves, int nr_scale_pr_octave);
    Eigen::Matrix4f alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int min_sample, int max_corr_dist);
    Eigen::Matrix4f icp_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr find_differences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, float distThresh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr statOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, int meanK, float stdDev);

};


#endif
