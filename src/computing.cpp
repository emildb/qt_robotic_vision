//
// Created by Emil Bjørlykhaug 06.04.16
//

#include "hahaha/computing.hpp"



Computing::Computing(){

}


std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> Computing::calculateVFHDescriptors(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters)
{
    std::cout << "Hi!" << std::endl;

    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> descriptors;

    for(int i=0;i<clusters.size();i++){
        // Object for storing the normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // Object for storing the VFH descriptor.
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

        // Estimate the normals.
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(clusters[i]);
        normalEstimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.compute(*normals);

        // VFH estimation object.
        pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
        vfh.setInputCloud(clusters[i]);
        vfh.setInputNormals(normals);
        vfh.setSearchMethod(kdtree);
        // Optionally, we can normalize the bins of the resulting histogram,
        // using the total number of points.
        vfh.setNormalizeBins(true);
        // Also, we can normalize the SDC with the maximum size found between
        // the centroid and any of the cluster's points.
        vfh.setNormalizeDistance(false);

        vfh.compute(*descriptor);
        descriptors.push_back(descriptor);
    }
    return descriptors;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Computing::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_filter)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    // Pass through filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_before_filter);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (0, 2.7);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2, 0);

    pass.filter (*cloud1);

    pass.setInputCloud(cloud1);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.45, 0.45);
    pass.filter (*cloud);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Computing::downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr  Computing::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);


    seg.setInputCloud (inCloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        //break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (inCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    return cloud_f;

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  Computing::extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (inCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inCloud);
    ec.extract (cluster_indices);


    for(int i = 0; i< cluster_indices.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*inCloud,cluster_indices[i],*tmpcloud);
        clusters.push_back(tmpcloud);
        std::cout << "PointCloud representing the Cluster: " << tmpcloud->size() << " data points." << std::endl;
    }
    return clusters;
}

pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr Computing::generate_search_tree(std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> inDescriptor)
{
    // Take global descriptors and put them in a kdtree

    pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor (new pcl::PointCloud<pcl::VFHSignature308>());
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree(new pcl::KdTreeFLANN<pcl::VFHSignature308>);
        for(int i = 0; i< inDescriptor.size(); i++){

        pcl::PointCloud<pcl::VFHSignature308>::Ptr temp_descriptor = inDescriptor.at(i);
        *global_descriptor += *(temp_descriptor);
    }


    search_tree->setInputCloud(global_descriptor);
    std::cout << "Size of tree: " << global_descriptor->size() << std::endl;
    return (search_tree);
}

std::vector<float> Computing::match_cloud(pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor, pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr inTree)
{
    std::vector<float> return_values;
    std::vector<int> best_match(1);
    std::vector<float> square_distance(1);

    inTree->nearestKSearch(global_descriptor->points[0],1,best_match,square_distance);
    return_values.push_back(best_match[0]);
    return_values.push_back(square_distance[0]);
    return return_values;
}

int Computing::search_for_correct_cluster(std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cluster_descriptors, std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cad_descriptors)
{
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr tree = generate_search_tree(cad_descriptors);

    float min_distance = 10000;
    int correct_cluster = 50;
    std::vector<float> search_result;

    for (int i = 0; i < cluster_descriptors.size(); i++){
        search_result = match_cloud(cluster_descriptors.at(i),tree);
        std::cout << search_result.at(1) << std::endl;
        if(search_result[1] < min_distance){
            min_distance = search_result[1];
            correct_cluster = i;
        }
    }
    std::cout << "Correct cluster: " << correct_cluster << std::endl;
    return correct_cluster;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Computing::compute_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                   pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint, float radius)
{
    pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_obj;
    fpfh_obj.setNumberOfThreads(8);
    fpfh_obj.setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    fpfh_obj.setRadiusSearch(radius);
    fpfh_obj.setSearchSurface(cloud);
    fpfh_obj.setInputNormals(normals);
    fpfh_obj.setInputCloud(keypoint);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_obj.compute(*descriptors);
    return descriptors;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Computing::generate_keypoints_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, float min_scale, int nr_octaves, int nr_scale_pr_octave)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*inCloud,*rgbcloud);
    for(int i = 0; i< rgbcloud->size(); i++){
        rgbcloud->points[i].r = 255;
        rgbcloud->points[i].g = 255;
        rgbcloud->points[i].b = 255;
    }

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    sift.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    sift.setScales(min_scale,nr_octaves,nr_scale_pr_octave);
    sift.setInputCloud(rgbcloud);
    sift.setMinimumContrast(0.0);
    pcl::PointCloud<pcl::PointWithScale> tmp;
    sift.compute(tmp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(tmp, *keyPoints);
    return keyPoints;
}

Eigen::Matrix4f Computing::alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int min_sample, int max_corr_dist)
{
    // Sample consensus alignment
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> initial_alignment;

    initial_alignment.setMinSampleDistance(min_sample);
    initial_alignment.setMaxCorrespondenceDistance(max_corr_dist);
    initial_alignment.setMaximumIterations(100);


    //Compute keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_key = generate_keypoints_sift(source_cloud,0.001,3,3);
    std::cout << source_cloud->size() << std::endl;
    std::cout << source_key->size() << std::endl;
    // Estimate the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation1;
    normalEstimation1.setInputCloud(source_cloud);
    normalEstimation1.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation1.setSearchMethod(kdtree1);
    normalEstimation1.compute(*normals1);
    //Compute local descriptors
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors = compute_FPFH(source_cloud, normals1, source_key,0.15);
    initial_alignment.setInputSource(source_key);
    initial_alignment.setSourceFeatures(source_descriptors);


    //Compute keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_key = generate_keypoints_sift(target_cloud,0.001,3,3);
    std::cout << target_cloud->size() << std::endl;
    std::cout << target_key->size() << std::endl;
    // Estimate the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation2;
    normalEstimation2.setInputCloud(target_cloud);
    normalEstimation2.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation2.setSearchMethod(kdtree2);
    normalEstimation2.compute(*normals2);
    //Compute local descriptors
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors = compute_FPFH(target_cloud, normals2, target_key,0.15);

    initial_alignment.setInputTarget(target_key);
    initial_alignment.setTargetFeatures(target_descriptors);


    pcl::PointCloud<pcl::PointXYZ> registration_output;
    initial_alignment.align(registration_output);

    std::cout << initial_alignment.getFinalTransformation() << std::endl;

    return (initial_alignment.getFinalTransformation());
}

Eigen::Matrix4f Computing::icp_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
    // Compute the ICP alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_b;
    icp_b.setInputSource(source_cloud);
    icp_b.setInputTarget(target_cloud);
    icp_b.setMaximumIterations(100);

    icp_b.align(*finalCloud);
    if (icp_b.hasConverged())
    {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp_b.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp_b.getFinalTransformation() << std::endl;
    }
    else std::cout << "ICP did not converge." << std::endl;
    return icp_b.getFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Computing::find_differences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, float distThresh)
{
    // The output cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::SegmentDifferences<pcl::PointXYZ> seg_diff;
    seg_diff.setSearchMethod(tree);
    seg_diff.setDistanceThreshold(distThresh);
    seg_diff.setInputCloud(source_cloud);
    seg_diff.setTargetCloud(target_cloud);
    seg_diff.segment(*segmentedCloud);
    return segmentedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Computing::statOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, int meanK, float stdDev)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "Cloud before filtering: " << std::endl;
    std::cout << *inCloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (inCloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stdDev);
    sor.filter (*cloud_filtered);

    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered << std::endl;

    return cloud_filtered;
}


