#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
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
std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
    int min_size, int max_size) {
    /* Euclidean clustering */
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr kd_tree(
        new pcl::search::KdTree<PointT>);
    kd_tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices_vector;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices_vector);

    for (const auto& cluster_indices : cluster_indices_vector) {
        typename pcl::PointCloud<PointT>::Ptr cluster(
            new pcl::PointCloud<PointT>);
        for (const auto& index : cluster_indices.indices) {
            cluster->push_back(cloud->at(index));
        }

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    return clusters;
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

template <typename PointT>
void RenderBoundingBox(pcl::visualization::PCLVisualizer::Ptr& viewer,
                       const typename pcl::PointCloud<PointT>::Ptr& cloud,
                       const std::string& name, Color color) {
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);

    /* Render box edge */
    std::string box_edge = name + "_box_edge";
    viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y,
                    min_point.z, max_point.z, color.r, color.g, color.b, box_edge);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_edge);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        box_edge);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, box_edge);

    /* Render box interior */
    std::string box_interior = name + "_box_interior";
    viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y,
                    min_point.z, max_point.z, color.r, color.g, color.b,
                    box_interior);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, box_interior);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        box_interior);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, box_interior);
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
                                     {0.5, 0.5, 0.5});

    /* Clustering */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters =
        Clustering<pcl::PointXYZI>(obstacle_cloud, 0.5, 100, 5000);

    /* Render Clusters */
    std::vector<Color> colors = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int cluster_id = 0;
    int color_index = 0;
    for (const auto& cluster : clusters) {
        RenderPointCloud<pcl::PointXYZI>(
            viewer, cluster, "cluster_" + std::to_string(cluster_id),
            colors.at(color_index));
        RenderBoundingBox<pcl::PointXYZI>(
            viewer, cluster, "cluster_" + std::to_string(cluster_id),
            colors.at(color_index));

        cluster_id += 1;
        color_index = (color_index + 1) % 3;
    }

    viewer->setCameraPosition(0, -35, 12, 0, 0, 1);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}