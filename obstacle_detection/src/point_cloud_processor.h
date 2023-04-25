#ifndef POINT_CLOUD_PROCESSOR_H_
#define POINT_CLOUD_PROCESSOR_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

struct Color {
    double r, g, b;
};

template <typename PointT>
std::vector<boost::filesystem::path> StreamPcd(const std::string& path) {
    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{path},
        boost::filesystem::directory_iterator{});
    std::sort(paths.begin(), paths.end());

    return paths;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr LoadPcd(const std::string& file) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) {
        return nullptr;
    }

    return cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Downsampling(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float resolution) {
    // voxel grid reduction
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(
        new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);
    voxel_grid_filter.filter(*filtered_cloud);

    return filtered_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr FiltRoi(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Vector4f& min_point, const Eigen::Vector4f& max_point) {
    // ROI filtering
    typename pcl::PointCloud<PointT>::Ptr roi_cloud(
        new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi_filter;
    roi_filter.setMin(min_point);
    roi_filter.setMax(max_point);
    roi_filter.setInputCloud(cloud);
    roi_filter.filter(*roi_cloud);

    // roof noise filtering
    roi_filter.setMin({-3.0, -3.0, -1.0, -1.0});
    roi_filter.setMax({3.0, 3.0, 1.0, 1.0});
    roi_filter.setInputCloud(roi_cloud);
    roi_filter.setNegative(true);
    roi_filter.filter(*roi_cloud);

    return roi_cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud,
             const int max_itertaions, const float distance_threshold) {
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
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float cluster_tolerance, const int min_size, const int max_size) {
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
                      const std::string& name, const Color& color) {
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
                       const std::string& name, const Color& color) {
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);

    /* Render box edge */
    std::string box_edge = name + "_box_edge";
    viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y,
                    min_point.z, max_point.z, color.r, color.g, color.b,
                    box_edge);
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
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
        box_interior);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        box_interior);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, box_interior);
}

#endif /* POINT_CLOUD_PROCESSOR_H_ */