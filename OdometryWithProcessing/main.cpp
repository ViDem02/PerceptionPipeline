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
#include <pcl/registration/default_convergence_criteria.h>

#include "odometry.h"

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("R = [\n");
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf(" %6.3f, %6.3f, %6.3f; \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("];\n");
    printf("t = [ %6.3f; %6.3f; %6.3f ];\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
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

    // Hand-tuned pitch corrections that level each scan.
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = level_and_filter(original_cloud_1, -10.0);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 = level_and_filter(original_cloud_2, -3.0);

    const MotionEstimate motion = estimate_motion(cloud_1, cloud_2, kIterations, kMaxCorrespondenceDistance);

    if (!motion.converged)
    {
        PCL_ERROR("ICP did not converge.\n");
        return -1;
    }

    // first.pcd moved onto second.pcd, for display
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_1, *cloud_1_aligned, motion.T_21.cast<float>());

    using CS = pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState;
    std::cout << "ICP finished in " << motion.ms << " ms with score " << motion.fitness << " ("
              << (motion.convergence_state == CS::CONVERGENCE_CRITERIA_ITERATIONS
                      ? "stopped at the iteration cap"
                      : "tolerance reached")
              << ")\n";
    std::cout << "Final transformation (first -> second):\n";
    print4x4Matrix(motion.T_21);

    // Visualization (static)
    pcl::visualization::PCLVisualizer viewer_1("ICP Result: first.pcd aligned to second.pcd");
    int vp_left(0), vp_right(1);
    viewer_1.createViewPort(0.0, 0.0, 0.5, 1.0, vp_left);
    viewer_1.createViewPort(0.5, 0.0, 1.0, 1.0, vp_right);


    float bg = 0.0f; float txt = 1.0f;
    viewer_1.setBackgroundColor(bg, bg, bg, vp_left);
    viewer_1.setBackgroundColor(bg, bg, bg, vp_right);

    // Color handlers
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_color(cloud_2, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(cloud_1, 20,180,20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(cloud_1_aligned, 200,30,30);

    viewer_1.addPointCloud(cloud_2, tgt_color, "cloud_in_left", vp_left);
    viewer_1.addPointCloud(cloud_1, src_color, "cloud_src_left", vp_left);

    viewer_1.addPointCloud(cloud_2, tgt_color, "cloud_in_right", vp_right);
    viewer_1.addPointCloud(cloud_1_aligned, aligned_color, "cloud_aligned_right", vp_right);

    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_left");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_src_left");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_right");
    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_aligned_right");

    // Overlay text
    std::ostringstream info_left;
    info_left << "Target: second.pcd (white)\nSource: first.pcd (green)\nMax iterations: " << kIterations;
    viewer_1.addText(info_left.str(), 10, 15, 14, txt, txt, txt, "info_left", vp_left);

    std::ostringstream info_right;
    info_right << "first.pcd aligned (red) vs second.pcd (white)\nScore: " << motion.fitness;
    viewer_1.addText(info_right.str(), 10, 15, 14, txt, txt, txt, "info_right", vp_right);

    // Transformation text (single line summary)
    std::ostringstream tf_line;
    tf_line << "Displacement: forward " << motion.displacement.x() << " m, right " << motion.displacement.z() << " m";
    viewer_1.addText(tf_line.str(), 10, 60, 14, txt, txt, txt, "tf_line", vp_right);

    viewer_1.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer_1.setSize(1280, 1024);

    // Vehicle displacement in the frame of the first scan (x forward, y up, z right).
    // With the README naming (x = East, z = North).
    const Eigen::Vector3d& d = motion.displacement;
    std::cout << "Displacement: forward (x, East) " << d.x() << " m, right (z, North) " << d.z()
              << " m, up (y) " << d.y() << " m; planar " << std::hypot(d.x(), d.z()) << " m\n";

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
