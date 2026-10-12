#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <QSurfaceFormat>
#include <iostream>
#include <thread>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>


#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

#include "logic/get_cones_cloud.h"
#include "logic/cones_recog.h"
#include "logic/distinguish_left_right.h"
#include "logic/get_obstacle_cloud.h"
#include "logic/racingline_nlopt.h"



void PCLViewer::doConnections() const
{

    connect(ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadButtonPressed ()));
    connect(ui->pushButton_compute, SIGNAL(clicked ()), this, SLOT(computeButtonPressed ()));





    connect(ui->horizontalSlider_sor, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));
    connect(ui->horizontalSlider_cutbottom, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));
    connect(ui->horizontalSlider_cuttop, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));
    connect(ui->horizontalSlider_lat, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));
    connect(ui->horizontalSlider_dist, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));
    connect(ui->horizontalSlider_fit, SIGNAL(sliderReleased ()), this, SLOT(slidersReleased ()));



    connect(ui->checkBox_originalcloud, SIGNAL(stateChanged (int)), this, SLOT(handleOriginalPointCloudCheckChanged(int)));
    connect(ui->checkBox_refcloud, SIGNAL(stateChanged (int)), this, SLOT(handleRefPointCloudCheckChanged(int)));



}




void PCLViewer::addPointCloudsToViewer() const
{

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_original_cloud(
        original_cloud,
        0,
        150,
        0);
    viewer->addPointCloud(original_cloud, handler_original_cloud, "original_cloud");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_racing_line(
        racing_line,
        255,
        200,
        0);
    viewer->addPointCloud(racing_line, handler_racing_line, "racing_line");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2, "racing_line");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_cones_cloud(
        cone_cloud,
        255,
        0,
        0);
    viewer->addPointCloud(cone_cloud, handler_cones_cloud, "cones_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cones_cloud");


    // Classified cone centres: left boundary blue, right boundary yellow.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_left_cones(
        left_cones,
        0,
        90,
        255);
    viewer->addPointCloud(left_cones, handler_left_cones, "left_cones");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,12,"left_cones");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_right_cones(
        right_cones,
        255,
        230,
        0);
    viewer->addPointCloud(right_cones, handler_right_cones, "right_cones");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,12,"right_cones");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_obst_cloud(
        obst_cloud,
        0,
        255,
        255);
    viewer->addPointCloud(obst_cloud, handler_obst_cloud, "obst_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"obst_cloud");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_reference_cloud(
        reference_cloud,
        255,
        255,
        255);
    viewer->addPointCloud(reference_cloud, handler_reference_cloud, "reference_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             15, "reference_cloud");



}


void PCLViewer::modelParameterSetting()
{
    initial_sor_std_dev_thr = 1.1;
    cone_detection_cut_bottom = -10.0;
    cone_detection_cut_top = -0.2;
    transl_on_y_axis = 0.0;
    dist_left_right_max_dist = 6;
    detect_cones_fitness_detection = 0.005;


    int coefficient = 0;
    double val = 0;
    double top = 0;
    double bottom = 0;

    ui->horizontalSlider_sor->setMaximum(static_cast<int>(initial_sor_std_dev_thr) * 100 + 100);
    ui->horizontalSlider_sor->setMinimum(static_cast<int>(initial_sor_std_dev_thr) * 100 - 50);
    ui->horizontalSlider_sor->setValue(initial_sor_std_dev_thr * 100);
    ui->lcdNumber_sor->display(initial_sor_std_dev_thr*100);

    val = cone_detection_cut_bottom;
    coefficient = 10;
    top = -5.0;
    bottom = -15.0;
    ui->horizontalSlider_cutbottom->setMaximum(static_cast<int>(top  * coefficient) );
    ui->horizontalSlider_cutbottom->setMinimum(static_cast<int>(bottom  * coefficient));
    ui->horizontalSlider_cutbottom->setValue(static_cast<int>(val * coefficient) );
    ui->lcdNumber_cutbottom->display(static_cast<int>(val * coefficient) );

    val = cone_detection_cut_top;
    coefficient = 10;
    top = 1.2;
    bottom = -10;
    ui->horizontalSlider_cuttop->setMaximum(static_cast<int>(top  * coefficient) );
    ui->horizontalSlider_cuttop->setMinimum(static_cast<int>(bottom  * coefficient));
    ui->horizontalSlider_cuttop->setValue(static_cast<int>(val * coefficient) );
    ui->lcdNumber_cuttop->display(static_cast<int>(val * coefficient) );

    val = transl_on_y_axis;
    coefficient = 10;
    top = 1;
    bottom = -1;
    ui->horizontalSlider_lat->setMaximum(static_cast<int>(top  * coefficient) );
    ui->horizontalSlider_lat->setMinimum(static_cast<int>(bottom  * coefficient));
    ui->horizontalSlider_lat->setValue(static_cast<int>(val * coefficient) );
    ui->lcdNumber_lat->display(static_cast<int>(val * coefficient) );


    val = dist_left_right_max_dist;
    coefficient = 10;
    top = 8;
    bottom = 0.1;
    ui->horizontalSlider_dist->setMaximum(static_cast<int>(top  * coefficient) );
    ui->horizontalSlider_dist->setMinimum(static_cast<int>(bottom  * coefficient));
    ui->horizontalSlider_dist->setValue(static_cast<int>(val * coefficient) );
    ui->lcdNumber_dist->display(static_cast<int>(val * coefficient) );


    val = detect_cones_fitness_detection;
    coefficient = 10000;
    top = 0.9;
    bottom = 0.0001;
    ui->horizontalSlider_fit->setMaximum(static_cast<int>(top  * coefficient) );
    ui->horizontalSlider_fit->setMinimum(static_cast<int>(bottom  * coefficient));
    ui->horizontalSlider_fit->setValue(static_cast<int>(val * coefficient) );
    ui->lcdNumber_fit->display(static_cast<int>(val * coefficient) );




}

PCLViewer::PCLViewer(QWidget *parent) : QMainWindow(parent),
                                        ui(new Ui::PCLViewer)
{
    // needed to ensure appropriate OpenGL context is created for VTK rendering.
    QSurfaceFormat::setDefaultFormat(PCLQVTKWidget::defaultFormat());

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");


    //TODO to be removed
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->resize(200);
    red = 128;
    green = 128;
    blue = 128;


    original_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ideal_cone_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    reference_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    working_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    obst_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    racing_line.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cone_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    left_cones.reset(new pcl::PointCloud<pcl::PointXYZ>);
    right_cones.reset(new pcl::PointCloud<pcl::PointXYZ>);

    reference_cloud->push_back(pcl::PointXYZ(1, 0, 0));
    reference_cloud->push_back(pcl::PointXYZ(-1, 0, 0));
    reference_cloud->push_back(pcl::PointXYZ(0, 0, 1));


    modelParameterSetting();


    // Set up the QVTK window
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->setRenderWindow(viewer->getRenderWindow().Get());
    viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());

#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
#endif

    doConnections();
    addPointCloudsToViewer();

    //viewer->resetCamera();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(-10, 0, 0, 10, 0, 0, 0,0,0);
    refreshView();
}


void
PCLViewer::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    ui->qvtkWidget->renderWindow()->Render();
#else
    ui->qvtkWidget->update();
#endif
}

void PCLViewer::updatePointClouds() const
{

    if (ui->checkBox_originalcloud->isChecked() == true)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_original_cloud(
            original_cloud,
            0,
            150,
            0);
        viewer->updatePointCloud(original_cloud, handler_original_cloud, "original_cloud");
    }


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_racing_line(
        racing_line,
        255,
        200,
        0);
    viewer->updatePointCloud(racing_line, handler_racing_line, "racing_line");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_cones_cloud(
        cone_cloud,
        255,
        0,
        0);
    viewer->updatePointCloud(cone_cloud, handler_cones_cloud, "cones_cloud");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_left_cones(
        left_cones,
        0,
        90,
        255);
    viewer->updatePointCloud(left_cones, handler_left_cones, "left_cones");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_right_cones(
        right_cones,
        255,
        230,
        0);
    viewer->updatePointCloud(right_cones, handler_right_cones, "right_cones");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_obst_cloud(
        obst_cloud,
        0,
        255,
        255);
    viewer->updatePointCloud(obst_cloud, handler_obst_cloud, "obst_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_reference_cloud(
        reference_cloud,
        255,
        255,
        255);
    viewer->updatePointCloud(reference_cloud, handler_reference_cloud, "reference_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             15, "reference_cloud");
}



void PCLViewer::loadButtonPressed()
{
    printf("Load button was pressed\n");

    const std::string file_name = ui->lineEdit_cones->text().toStdString();
    const std::string ideal_cone_model_name = ui->lineEdit_ideal_cone->text().toStdString();

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(ideal_cone_model_name, *ideal_cone_model) == -1)
    {
        PCL_ERROR("Couldn't read the cone model PLY file.\n");
        ui->label_errors->setText("Couldn't read the ideal cone model PLY file.");
        return;
    }

    try
    {
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ>(file_name, *original_cloud);
    }catch (const std::exception& e)
    {
        ui->label_errors->setText(e.what());
        return;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, 0.1;
    transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(-M_PI / 1, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*original_cloud, *original_cloud, transform);

    clouds_loaded = true;

    ui->label_errors->setText("");
}


void PCLViewer::computeButtonPressed()
{
    if (clouds_loaded == false)
    {
        ui->label_errors->setText("Please load the clouds first.");
        return;
    }

    working_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    obst_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    racing_line.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cone_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    left_cones.reset(new pcl::PointCloud<pcl::PointXYZ>);
    right_cones.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*original_cloud, *working_cloud);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, transl_on_y_axis;
    pcl::transformPointCloud(*working_cloud, *working_cloud, transform);
    pcl::transformPointCloud(*cone_cloud, *cone_cloud, transform);

    const bool verbose = ui->checkBox_verbose->isChecked();

    auto [centers_of_mass, new_cone_cloud] = get_cones_cloud(
        working_cloud,
        cone_cloud,
        initial_sor_std_dev_thr,
        cone_detection_cut_bottom,
        cone_detection_cut_top,
        detect_cones_fitness_detection, verbose, ideal_cone_model
    );



    if (! cone_cloud->empty())
    {
        auto [src_left, src_right] =
            cones::distinguishLeftRight(
                centers_of_mass,
                0,
                0,
                -45,
                +45,
                dist_left_right_max_dist);

        left_cones = src_left;
        right_cones = src_right;

        racing_line = get_racing_line_point_cloud(
            src_left,
            src_right);

        obst_cloud = get_obstacle_cloud(working_cloud, cone_cloud);
    }

    updatePointClouds();
    refreshView();
}

void PCLViewer::slidersReleased()
{
    initial_sor_std_dev_thr = ui->horizontalSlider_sor->value() / 100.0;
    cone_detection_cut_bottom = ui->horizontalSlider_cutbottom->value() / 10.0;
    cone_detection_cut_top = ui->horizontalSlider_cuttop->value() / 10.0;
    transl_on_y_axis = ui->horizontalSlider_lat->value() / 10.0;
    dist_left_right_max_dist = ui->horizontalSlider_dist->value() / 10.0;;
    detect_cones_fitness_detection = ui->horizontalSlider_fit->value() / 10000.0;;

    computeButtonPressed();
}

void PCLViewer::handleOriginalPointCloudCheckChanged(int state)
{
    if (state == Qt::Checked)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_original_cloud(
        original_cloud,
        0,
        150,
        0);
        viewer->addPointCloud(original_cloud, handler_original_cloud, "original_cloud");
    }
    else
    {
        viewer->removePointCloud("original_cloud");
    }
    refreshView();
}

void PCLViewer::handleRefPointCloudCheckChanged(int state)
{
    if (state == Qt::Checked)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_reference_cloud(
        reference_cloud,
        255,
        255,
        255);
        viewer->addPointCloud(reference_cloud, handler_reference_cloud, "reference_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                 15, "reference_cloud");
    }
    else
    {
        viewer->removePointCloud("reference_cloud");
    }
    refreshView();
}


PCLViewer::~PCLViewer()
{
    delete ui;
}
