#include <iostream>
#include <cstdlib>
#include <thread>
#include <vector>


#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("R = [\n");
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("];\n");
    printf("t = [ %6.3f; %6.3f; %6.3f ];\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}




// Minimal helper returning just north and east from a 4x4 transform.
// Axes convention: x = East, y = Up, z = North.
// If transformMapsSecondToFirst is true, T maps second->first and the motion first->second is inv(T).
// Set invertMotion to true to flip the direction after resolving the frame mapping.
struct NorthEast { double north; double east; };
static NorthEast computeNorthEast(
    const Eigen::Matrix4d &T,
    bool transformMapsSecondToFirst,
    bool invertMotion)
{
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Vector3d d = transformMapsSecondToFirst ? (-(R.transpose()) * t) : t; // first->second
    if (invertMotion) d = -d;
    return NorthEast{d.z(), d.x()};
}



int main(int argc, char** argv)
{
    std::string file_name_1 = "first.pcd";
    std::string file_name_2 = "second.pcd";
    if (argc > 2) {
        file_name_1 = argv[1];
        file_name_2 = argv[2];
    } else {
        std::cout << "Usage: " << argv[0] << " <file_name_1.pcd> <file_name_2.pcd>\n";
        return 1;
    }
    
    pcl::PCDWriter writer;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_t(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    try {
        if (reader.read<pcl::PointXYZ>(file_name_1, *original_cloud_1) < 0) {
            throw std::runtime_error("Failed to read " + file_name_1);
        }
        if (reader.read<pcl::PointXYZ>(file_name_2, *original_cloud_2) < 0) {
            throw std::runtime_error("Failed to read " + file_name_2);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading PCD files: " << e.what() << std::endl;
        return 2;
    }

    pcl::copyPointCloud(*original_cloud_1, *cloud_1);
    pcl::copyPointCloud(*original_cloud_2, *cloud_2);

    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.translation() << 0.0, 0.0, 0.0;
    transform_1.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    transform_1.rotate(Eigen::AngleAxisf(-10.0 * M_PI/180, Eigen::Vector3f::UnitY()));
    
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(-3.0 * M_PI/180, Eigen::Vector3f::UnitY()));


    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_1);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1, 10);
    pass.filter(*cloud_1);

    pass.setInputCloud(cloud_2);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1, 10);
    pass.filter(*cloud_2);

    pcl::transformPointCloud(*cloud_1, *cloud_1, transform_1);
    pcl::transformPointCloud(*cloud_2, *cloud_2, transform_2);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_1);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_1);


    sor.setInputCloud(cloud_2);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_2);

    constexpr int iterations = 70;

    pcl::console::TicToc time; time.tic();

    // Run ICP once for the requested iterations.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_1);
    icp.setInputTarget(cloud_2);
    icp.setMaximumIterations(iterations);
    time.tic();
    icp.align(*cloud_2_t);
    double ms = time.toc();

    if (!icp.hasConverged())
    {
        PCL_ERROR("ICP did not converge.\n");
        return -1;
    }

    Eigen::Matrix4d final_transform = icp.getFinalTransformation().cast<double>();
    std::cout << "ICP converged in " << ms << " ms with score " << icp.getFitnessScore() << "\n";
    std::cout << "Final transformation (second -> first):\n";
    print4x4Matrix(final_transform);


    // Visualization (static)
    pcl::visualization::PCLVisualizer viewer_1("ICP Result: second.pcd aligned to first.pcd");
    int vp_left(0), vp_right(1);
    viewer_1.createViewPort(0.0, 0.0, 0.5, 1.0, vp_left);
    viewer_1.createViewPort(0.5, 0.0, 1.0, 1.0, vp_right);


    float bg = 0.0f; float txt = 1.0f;
    viewer_1.setBackgroundColor(bg, bg, bg, vp_left);
    viewer_1.setBackgroundColor(bg, bg, bg, vp_right);

    // Color handlers
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_color(cloud_1, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(cloud_2, 20,180,20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(cloud_2_t, 200,30,30);

    viewer_1.addPointCloud(cloud_1, tgt_color, "cloud_in_left", vp_left);
    viewer_1.addPointCloud(cloud_2, src_color, "cloud_src_left", vp_left);

    viewer_1.addPointCloud(cloud_1, tgt_color, "cloud_in_right", vp_right);
    viewer_1.addPointCloud(cloud_2_t, aligned_color, "cloud_aligned_right", vp_right);

    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_left");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_src_left");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_right");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_aligned_right");

    // Overlay text
    std::ostringstream info_left;
    info_left << "Target: first.pcd (white)\nSource: second.pcd (green)\nIterations: " << iterations;
    viewer_1.addText(info_left.str(), 10, 15, 14, txt, txt, txt, "info_left", vp_left);

    std::ostringstream info_right;
    info_right << "Result (red) vs target (white)\nScore: " << icp.getFitnessScore();
    viewer_1.addText(info_right.str(), 10, 15, 14, txt, txt, txt, "info_right", vp_right);

    // Transformation text (single line summary)
    std::ostringstream tf_line;
    tf_line << "Translation: (" << final_transform(0,3) << ", " << final_transform(1,3) << ", " << final_transform(2,3)
            << ")";
    viewer_1.addText(tf_line.str(), 10, 60, 14, txt, txt, txt, "tf_line", vp_right);

    viewer_1.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer_1.setSize(1280, 1024);

    auto res = computeNorthEast(final_transform, true, true);

    std::cout << " North: " << res.north << " East: " << res.east << "\n";

    if (std::getenv("PERCEPTION_HEADLESS") != nullptr)
    {
        return 0;
    }

    // Static visualization loop (no animation)
    while (!viewer_1.wasStopped())
    {
        viewer_1.spinOnce(30);
    }
    return 0;

}
