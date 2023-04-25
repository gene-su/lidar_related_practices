#include "point_cloud_processor.h"

template <typename PointT>
void ProcessPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                       const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    /* Downsampling */                    
    typename pcl::PointCloud<PointT>::Ptr downsampling_cloud =
        Downsampling<PointT>(cloud, 0.3);

    /* ROI Filtering */
    typename pcl::PointCloud<PointT>::Ptr roi_cloud =
        FiltRoi<PointT>(downsampling_cloud, {-10, -5, -2, 1}, {30, 8, 1, 1});

    /* Segmentation */
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
        segmented_clouds = SegmentPlane<PointT>(roi_cloud, 100, 0.2);
    RenderPointCloud<PointT>(viewer, segmented_clouds.first, "road_cloud",
                             {0.5, 0.5, 0.5});

    /* Clustering */
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters =
        Clustering<PointT>(segmented_clouds.second, 0.5, 100, 5000);

    /* Render Clusters */
    std::vector<Color> colors = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int cluster_id = 0;
    int color_index = 0;
    for (const auto& cluster : clusters) {
        RenderPointCloud<PointT>(viewer, cluster,
                                 "cluster_" + std::to_string(cluster_id),
                                 colors.at(color_index));
        RenderBoundingBox<PointT>(viewer, cluster,
                                  "cluster_" + std::to_string(cluster_id),
                                  colors.at(color_index));

        cluster_id += 1;
        color_index = (color_index + 1) % 3;
    }
}

int main(int argc, char** argv) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("viewer"));

    // ProcessPointClouds<pcl::PointXYZI> process_point_cloud;

    std::vector<boost::filesystem::path> pcd_stream =
        StreamPcd<pcl::PointXYZI>("../src/data/data1");
    auto pcd_stream_iter = pcd_stream.begin();

    viewer->setCameraPosition(-15, 0, 0, 0, 0, 1);
    while (!viewer->wasStopped()) {
        // Clear Viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load .pcd File
        if (pcd_stream_iter == pcd_stream.end()) {
            pcd_stream_iter = pcd_stream.begin();
        }

        // Process Point Cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud =
            LoadPcd<pcl::PointXYZI>((*pcd_stream_iter).string());
        if (input_cloud == nullptr) {
            return 0;
        }

        ProcessPointCloud<pcl::PointXYZI>(viewer, input_cloud);

        pcd_stream_iter++;
        viewer->spinOnce();
    }
}