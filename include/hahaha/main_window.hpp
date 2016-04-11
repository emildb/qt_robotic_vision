/**
 * @file /include/hahaha/main_window.hpp
 *
 * @brief Qt based gui for hahaha.
 *
 * @date November 2010
 **/
#ifndef hahaha_MAIN_WINDOW_H
#define hahaha_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

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
#include "hahaha/computing.hpp"



/*****************************************************************************
** Namespace
*****************************************************************************/

namespace hahaha {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


    void closeEvent(QCloseEvent *event); // Overloaded function



public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
    *******************************************/

    void updateTopics(QStringList list);

    void on_pushButton_clicked(bool check);

    void on_pushButton_2_clicked(bool check);

    void on_button_take_pic_clicked(bool check);

    void on_connect_robot_button_clicked(bool check);

    void on_move_robot_button_clicked(bool check);

    void on_pushButton_5_clicked(bool check);

    void on_pushButton_6_clicked(bool check);

    void on_insert_cloud_button_clicked(bool check);

    void on_insert_cloud_segment_button_clicked(bool check);

    void on_descriptors_button_clicked(bool check);

    void on_segment_two_clouds_button_clicked(bool check);

    void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c);
    void addCADinPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c, std::string, int color);
    void getPointCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n);
    void getPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);

    /******************************************
    ** Manual connections
    *******************************************/

Q_SIGNALS:
    void getTopics();

    void subscribeToPointCloud2(QString topic);

    void takePictures(int nrPictures, QString url, bool display);

    void connect_agilus_robot();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QVTKWidget *w;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    RayTraceLoader *rayTraceLoader;
    Computing *compute;
};

}  // namespace hahaha

#endif // hahaha_MAIN_WINDOW_H
