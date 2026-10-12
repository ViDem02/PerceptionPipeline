#pragma once

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

/*
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
*/


namespace Ui
{
    class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    void doConnections() const;

    void addPointCloudsToViewer() const;

    void modelParameterSetting();

    explicit PCLViewer(QWidget *parent = 0);

    ~PCLViewer();

public Q_SLOTS:

    void
    loadButtonPressed();

    void
    computeButtonPressed();


    //sliders

    void
    slidersReleased();

    void
    handleOriginalPointCloudCheckChanged(int state);

    void
    handleRefPointCloudCheckChanged(int state);


protected:
    void
    refreshView();

    void
    updatePointClouds() const;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

    unsigned int red;
    unsigned int green;
    unsigned int blue;

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ideal_cone_model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obst_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr racing_line;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cones;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cones;

    bool clouds_loaded = false;

    double initial_sor_std_dev_thr;
    float cone_detection_cut_bottom;
    float cone_detection_cut_top;
    float transl_on_y_axis;
    float dist_left_right_max_dist;
    double detect_cones_fitness_detection;

private:
    Ui::PCLViewer *ui;

};
