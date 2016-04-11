/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/hahaha/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hahaha {

using namespace Qt;

//Function used in uniform sampling
inline double uniform_deviate(int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

//Function used in uniform sampling
inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
    Eigen::Vector4f& p)
{
    float r1 = static_cast<float> (uniform_deviate(rand()));
    float r2 = static_cast<float> (uniform_deviate(rand()));
    float r1sqr = sqrtf(r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}

//Function used in uniform sampling
inline void randPSurface(vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
    float r = static_cast<float> (uniform_deviate(rand()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
    vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    polydata->GetCellPoints(el, npts, ptIds);
    polydata->GetPoint(ptIds[0], A);
    polydata->GetPoint(ptIds[1], B);
    polydata->GetPoint(ptIds[2], C);
    randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
        float(B[0]), float(B[1]), float(B[2]),
        float(C[0]), float(C[1]), float(C[2]), p);
}

//Transforms a vtkSmartPointer to a PointCloud with a given amount of points
void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> &
    cloud_out)
{
    polydata->BuildCells();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();
    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector <double> cumulativeAreas(cells->GetNumberOfCells(), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++) {
        polydata->GetPoint(ptIds[0], p1);
        polydata->GetPoint(ptIds[1], p2);
        polydata->GetPoint(ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }

    cloud_out.points.resize(n_samples);
    cloud_out.width = static_cast<pcl::uint32_t > (n_samples);
    cloud_out.height = 1;
    for (i = 0; i < n_samples; i++) {
        Eigen::Vector4f p;
        randPSurface(polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this, SIGNAL(getTopics()), &qnode, SLOT(findTopics()));
    QObject::connect(&qnode, SIGNAL(sendTopics(QStringList)), this, SLOT(updateTopics(QStringList)));
    QObject::connect(&qnode, SIGNAL(setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)), this, SLOT(getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)));
    QObject::connect(&qnode, SIGNAL(setPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)), this, SLOT(getPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)));
    QObject::connect(this, SIGNAL(subscribeToPointCloud2(QString)), &qnode, SLOT(subscribeToPointCloud2(QString)));
    QObject::connect(this, SIGNAL(takePictures(int, QString, bool)), &qnode, SLOT(takePicture(int, QString, bool)));
    QObject::connect(this, SIGNAL(connect_agilus_robot()), &qnode, SLOT(connect_agilus()));

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
    rayTraceLoader = new RayTraceLoader("Test");
    rayTraceLoader->populateLoader();



    w = new QVTKWidget();
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    w->SetRenderWindow(viewer->getRenderWindow());
    w->update();
    QHBoxLayout *layout = new QHBoxLayout;
    ui.frame_3->setLayout(layout);
    ui.frame_3->layout()->addWidget(w);

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::updateTopics(QStringList list)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(list);
}


void MainWindow::on_pushButton_2_clicked(bool check)
{
    if(ui.comboBox->currentText().length() != 0){
        ui.pushButton_2->setEnabled(true);
        Q_EMIT subscribeToPointCloud2(ui.comboBox->currentText());
    }
}

void MainWindow::on_pushButton_clicked(bool check)
{
    Q_EMIT getTopics();
}

void MainWindow::on_button_take_pic_clicked(bool check)
{
    QString savefile = QFileDialog::getSaveFileName(this, tr("savePictureString"), "/home/minions/Workspaces/PointClouds/",tr("PointCloud (*.pcd"));
    //QString url = ui.textbox_path->toPlainText();
    //url.append("/");
    //url.append(ui.fileName->text());
    Q_EMIT takePictures(1, savefile, false);
}

void MainWindow::on_connect_robot_button_clicked(bool check)
{
    Q_EMIT connect_agilus_robot();
}

void MainWindow::on_move_robot_button_clicked(bool check)
{

}

void MainWindow::on_pushButton_5_clicked(bool check){
    cout << "Button clicked!" << std::endl;
    QString modelName;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    modelName = QFileDialog::getOpenFileName(this,tr("Choose MODEL cloud"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::io::loadPCDFile(modelName.toStdString(), *cloud_before_filter);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_before_filter);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10000000, 1.1);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

    getPointCloudNormal(cloud, normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.1);

    // Set typical values for the parameters
    gp3.setMu (3.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    pcl::io::savePolygonFileSTL("/home/minions/rofl2.stl",triangles);
    pcl::io::saveVTKFile("/home/minions/rofl.vtk",triangles);


    QFile file("/home/minions/rofl2.stl");
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);

    QString text;

    while(!in.atEnd()) {
        QString line = in.readLine();
        line.replace(",",".");

        text.append(line);
        text.append("\n");
    }

    file.close();

    //std::cout << text.toStdString() << std::endl;


    QFile output_file("/home/minions/updated.stl");
    output_file.open(QIODevice::WriteOnly);
    QTextStream roflcopter(&output_file);
    roflcopter << text;
    output_file.close();
    std::cout << "ferdig me ny fil" << std::endl;

}

void MainWindow::on_pushButton_6_clicked(bool check)
{
    cout << "Button clicked!" << std::endl;
    QString modelName;
    pcl::PolygonMesh mesh;
    modelName = QFileDialog::getOpenFileName(this,tr("Choose MODEL cloud"),"/home/",tr("STL File (*.stl *.STL)"));
    pcl::io::loadPolygonFileSTL(modelName.toStdString(), mesh);
    std::cout << "Loaded file." << std::endl;





    pcl::PointCloud<pcl::PointXYZ> temp_trans_cloud;
    fromPCLPointCloud2(mesh.cloud, temp_trans_cloud);
    Eigen::Matrix4f scaleCAD = Eigen::Matrix4f::Identity();
    float scaleFactor = 0.001;
    scaleCAD(0, 0) = scaleFactor;
    scaleCAD(0, 1) = 0;
    scaleCAD(1, 1) = scaleFactor;
    scaleCAD(1, 0) = 0;
    scaleCAD(2, 2) = scaleFactor;
    scaleCAD(0, 3) = 0;
    scaleCAD(1, 3) = 0;
    scaleCAD(2, 3) = 0;
    pcl::transformPointCloud(temp_trans_cloud, temp_trans_cloud, scaleCAD);
    toPCLPointCloud2(temp_trans_cloud, mesh.cloud);

    rayTraceLoader = new RayTraceLoader(mesh, "Test");
    rayTraceLoader->setCloudResolution(200);
    rayTraceLoader->populateLoader();


    pcl::PointCloud<pcl::PointXYZ>::Ptr platePC(new pcl::PointCloud<pcl::PointXYZ>);

    vtkSmartPointer<vtkPolyData> meshVTK2;
    pcl::VTKUtils::convertToVTK(mesh, meshVTK2);
    uniform_sampling(meshVTK2, 5000, *platePC);

    addCADinPointCloud(platePC, "cad_1",10);

}

void MainWindow::on_insert_cloud_button_clicked(bool check)
{
    cout << "Button clicked!" << std::endl;
    cout << "Button clicked rofl!" << std::endl;
    QString modelName;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    modelName = QFileDialog::getOpenFileName(this,tr("Choose point cloud"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::io::loadPCDFile(modelName.toStdString(), *cloud_before_filter);

    addCADinPointCloud(cloud_before_filter, modelName.toStdString(),100);
}

void MainWindow::on_insert_cloud_segment_button_clicked(bool check)
{
    cout << "Button clicked!" << std::endl;
    QString modelName;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_filter (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    modelName = QFileDialog::getOpenFileName(this,tr("Choose point cloud"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::io::loadPCDFile(modelName.toStdString(), *cloud_before_filter);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Pass through filter to just get the table
    cloud = compute->passThroughFilter(cloud_before_filter);

    // To see if the downsampling has worked. There is a bug in the library cause too small leaf size not to work
    std::cout << "PointCloud before filtering has: " << cloud->points.size ()  << " data points." << std::endl; //*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = compute->downsample_voxel(cloud, 0.007f);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Segmentation to get rid of the table
    cloud_filtered =  compute->segmentation(cloud_filtered);

    // Extract clusters of points. This then makes us able to find out which cluster we are interested in
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = compute->extract_clusters(cloud_filtered);


    // Calculate VFH descriptors for the clusters from the point cloud
    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> descriptors = compute->calculateVFHDescriptors(clusters);

    std::cout << descriptors.size() << std::endl;

    // Ray trace the saved model
    std::vector<RayTraceCloud> rayTraceList = rayTraceLoader->getPointClouds(true);
    std::cout << rayTraceList.size() << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cad_clusters;

    // Add the clusters in a vector
    for(int i=0;i<rayTraceList.size();i++){
        cad_clusters.push_back(rayTraceList.at(i).cloud);
    }
    // Calculate the VFH descriptors for clusters of the cad model
    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cad_descriptors = compute->calculateVFHDescriptors(cad_clusters);
    std::cout << cad_descriptors.size() << std::endl;

    // Find the which of the clusters in the point cloud we are interested in
    int corr;
    corr = compute->search_for_correct_cluster(descriptors,cad_descriptors);

    // Do an initial alignment with feature detection
    Eigen::Matrix4f initAlignment = compute->alignment(cad_clusters[corr],cloud_filtered,0.01,1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr initModel(new pcl::PointCloud<pcl::PointXYZ>());

    // Transform the point cloud according to the initial alignment (red color)
    pcl::transformPointCloud(*cad_clusters[corr], *initModel, initAlignment);
    // Visualize it
    addCADinPointCloud(initModel, "one",0);

    // Do the final transformation with ICP
    Eigen::Matrix4f finalAlignment = compute->icp_alignment(initModel,cloud_filtered);
    // Add the transformations to get it from start to finish
    Eigen::Matrix4f totAlignment  = finalAlignment*initAlignment;

    std::cout << "Total alignment matrix:" << std::endl;
    std::cout << totAlignment << std::endl;

    // Transform the model according to the total transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalModel(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cad_clusters[corr], *finalModel, totAlignment);

    // Visualize the total transformation in blue
    addCADinPointCloud(finalModel, "two",1);

    // Visualize the original cloud without the table
    addCADinPointCloud(cloud_filtered, "clusters",10);

}

void MainWindow::on_descriptors_button_clicked(bool check)
{
    //TODO matche descriptors. lage og lagre descriptors når load CAD.
}

void MainWindow::on_segment_two_clouds_button_clicked(bool check)
{
    cout << "Button clicked!" << std::endl;
    QString modelName1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    modelName1 = QFileDialog::getOpenFileName(this,tr("Choose point cloud 1"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::io::loadPCDFile(modelName1.toStdString(), *cloud1);
    cloud1_filtered = compute->statOutlierRemoval(cloud1, 50, 0.7);

    QString modelName2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    modelName2 = QFileDialog::getOpenFileName(this,tr("Choose point cloud 2"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::io::loadPCDFile(modelName2.toStdString(), *cloud2);
    cloud2_filtered = compute->statOutlierRemoval(cloud2, 50, 0.7);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZ>);

    //cout << "before compute!" << std::endl;

    cloud_seg = compute->find_differences(cloud1_filtered,cloud2_filtered, 0.001);

    //cout << "After compute!" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    cloudSegFiltered = compute->statOutlierRemoval(cloud_seg,50,0.7);


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = compute->extract_clusters(cloudSegFiltered);
    for(int i=0;i<clusters.size();i++){
        if(clusters.at(i)->size()>500)
        std::cout << "Found something that is not meant to be there!" << std::endl;
    }


    addCADinPointCloud(clusters.at(0), "hihi",10);

}



void MainWindow::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c)
{

    if(!viewer->updatePointCloud(c, "cloud")){
        viewer->addPointCloud(c, "cloud");
        w->update();
    }
    w->update();
}

void MainWindow::addCADinPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c, std::string name, int color)
{
    if(color==0){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (c,255,0,0);
        viewer->addPointCloud(c,red,name);
    }
    else if(color == 1){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (c,0,0,255);
        viewer->addPointCloud(c,blue,name);
    }
    else{
        viewer->addPointCloud(c, name);
    }
    w->update();
}

void MainWindow::getPointCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n)
{

    if(!viewer->updatePointCloud(c, "cloud")){
        viewer->addPointCloud(c, "cloud");
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(c,n,10,0.05, "normals");
        w->update();
    }
    //w->update();
}



void MainWindow::getPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c)
{
    if(!viewer->updatePointCloud(c, "cloud")){
        viewer->addPointCloud(c, "cloud");
        w->update();
    }
    w->update();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/



/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/



void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace hahaha

