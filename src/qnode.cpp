/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/hahaha/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hahaha {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"hahaha");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(200);
    while ( ros::ok() ) {

		ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::findTopics(){
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/PointCloud2", Qt::CaseInsensitive) == 0){
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    Q_EMIT sendTopics(list);
}

void QNode::subscribeToPointCloud2(QString topic){
    ros::NodeHandle n;
    //rgb_enabled = rgb;
    const char *tmp = topic.toUtf8().constData();
    pointCloud2Sub = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1, &QNode::cloudCallback,this);
}


void QNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
        pcl::fromROSMsg(*cloud_msg, cloud);
        //if(rgb_enabled){
            pcl::fromROSMsg(*cloud_msg, cloudRGB);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*cloud_msg, *tmpCloud);
            Q_EMIT setPointCloudRGB(tmpCloud);
        //}
        //else{
            // Cloud conversion and visualization
        //    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>);
        //    pcl::fromROSMsg(*cloud_msg, *tmpCloud);
        //    Q_EMIT setPointCloud(tmpCloud);
        //}
        picture_flag = true;

}

void QNode::takePicture(int nrPicture, QString url, bool display)
{
    bool running = true;
    int picture_taken = 0;
    QString tmpUrl;
    rgb_enabled = true;
    while(running)
    {
        if(picture_flag)
        {
            tmpUrl = url;
            tmpUrl.append("_");
            tmpUrl.append(QString::number(picture_taken+1));
            tmpUrl.append(".pcd");
            //take picture here
            if(rgb_enabled){
                //pcl::io::savePCDFile(tmpUrl.toUtf8().constData(), cloudRGB);
                pcl::io::savePCDFileBinary(tmpUrl.toUtf8().constData(),cloudRGB);

                picture_taken = picture_taken + 1;
            }
            else{
                pcl::io::savePCDFileASCII(tmpUrl.toUtf8().constData(), cloud);
                picture_taken = picture_taken + 1;
            }
            picture_flag = false;
        }
        if(picture_taken == nrPicture)
        {
            running = false;
        }
    }
    //if(display){
    //    Q_EMIT displayImage(tmpUrl);
    //}
}

void QNode::connect_agilus()
{
    ros::NodeHandle node_handle;
    std::cout << "Output sentence" << std::endl;
    moveit::planning_interface::MoveGroup group("agilus1");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(
            group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = 0.0;
    group_variable_values[1] = -M_PI / 2.0;
    group_variable_values[2] = M_PI / 2.0;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = M_PI / 2.0;
    group_variable_values[5] = 0.0;
    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
}



}  // namespace hahaha
