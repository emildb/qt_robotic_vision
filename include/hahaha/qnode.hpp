/**
 * @file /include/hahaha/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hahaha_QNODE_HPP_
#define hahaha_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "stdio.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hahaha {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
	void run();


    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

public Q_SLOTS:
    void findTopics();

    void subscribeToPointCloud2(QString topic);

    void takePicture(int nrPicture, QString url, bool display);

    void connect_agilus();

Q_SIGNALS:
    void rosShutdown();
    void sendTopics(QStringList list);

    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c);
    void setPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);

private:
	int init_argc;
    char** init_argv;

    pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ros::Subscriber pointCloud2Sub;

    bool picture_flag;
    bool rgb_enabled;
};

}  // namespace hahaha

#endif /* hahaha_QNODE_HPP_ */
