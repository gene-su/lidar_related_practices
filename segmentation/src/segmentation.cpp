#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

struct Color {
    double r, g, b;
};

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr LoadPcd(const std::string& file) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) {
        return nullptr;
    }

    return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_itertaions,
             float distance_threshold) {
    /* RANSAC segmentation */
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_itertaions);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inlier_indices, *coefficients);

    typename pcl::PointCloud<PointT>::Ptr plane_cloud(
        new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(
        new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extraction;
    extraction.setInputCloud(cloud);
    extraction.setIndices(inlier_indices);
    extraction.setNegative(false);
    extraction.filter(*plane_cloud);
    extraction.setNegative(true);
    extraction.filter(*obstacle_cloud);

    return std::make_pair(plane_cloud, obstacle_cloud);
}

template <typename PointT>
void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const typename pcl::PointCloud<PointT>::Ptr& cloud,
                      const std::string& name, Color color) {
    viewer->addPointCloud<PointT>(cloud, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        name);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

int main(int argc, char** argv) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("viewer"));

    /* Load .pcd file */
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud =
        LoadPcd<pcl::PointXYZI>("../src/data/test.pcd");
    if (input_cloud == nullptr) {
        return 0;
    }

    /* Segmentation */
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              pcl::PointCloud<pcl::PointXYZI>::Ptr>
        result_clouds = SegmentPlane<pcl::PointXYZI>(input_cloud, 100, 0.2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud = result_clouds.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud = result_clouds.second;
    RenderPointCloud<pcl::PointXYZI>(viewer, plane_cloud, "plane_cloud",
                                     {1, 0, 0});
    RenderPointCloud<pcl::PointXYZI>(viewer, obstacle_cloud, "obstacle_cloud",
                                     {0, 1, 0});

    viewer->setCameraPosition(0, -35, 12, 0, 0, 1);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}