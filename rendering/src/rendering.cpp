#include <pcl/visualization/pcl_visualizer.h>

#include "car.h"
#include "common.h"
#include "lidar.h"

void RenderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    /* road */
    std::string id = "highway";
    double road_length = 50.0;
    double road_width = 12.0;
    double road_height = 0.2;
    Color road_color{0.2, 0.2, 0.2};
    viewer->addCube(-road_length / 2., road_length / 2., -road_width / 2.,
                    road_width / 2., -road_height, 0., road_color.r,
                    road_color.g, road_color.b, id);
    // representation: points, wireframe or surface
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);

    /* line */
    Color line_color{1.0, 1.0, 0.};
    viewer->addLine(pcl::PointXYZ(-road_length / 2, -road_width / 6, 0.01),
                    pcl::PointXYZ(road_length / 2, -road_width / 6, 0.01),
                    line_color.r, line_color.g, line_color.b, "line1");
    viewer->addLine(pcl::PointXYZ(-road_length / 2, road_width / 6, 0.01),
                    pcl::PointXYZ(road_length / 2, road_width / 6, 0.01),
                    line_color.r, line_color.g, line_color.b, "line2");
}

void RenderRays(pcl::visualization::PCLVisualizer::Ptr& viewer,
                const Eigen::Vector3d& origin,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    static int ray_count = 0;
    for (pcl::PointXYZ point : cloud->points) {
        viewer->addLine(pcl::PointXYZ(origin.x(), origin.y(), origin.z()),
                        point, 1, 0, 0, "ray" + std::to_string(ray_count));
        ray_count += 1;
    }
}

void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      std::string name, Color color) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        name);
}

int main(int argc, char** argv) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D viewer"));

    RenderHighway(viewer);

    Car my_car("my_car", {0., 0., 0.}, {4., 2., 2.}, {0., 1., 0.}, 1.);
    Car car1("car1", {15., 0., 0.}, {4., 2., 2.}, {0., 0., 1.}, 0.3);
    Car car2("car2", {8., -4., 0.}, {4., 2., 2.}, {0., 0., 1.}, 0.3);
    Car car3("car3", {-12., 4., 0.}, {4., 2., 2.}, {0., 0., 1.}, 0.3);
    std::vector<Car> cars{my_car, car1, car2, car3};
    for (const auto& it : cars) {
        it.Render(viewer);
    }

    Lidar lidar({0, 0, 2.6}, 5, 50, 5. * M_PI / 180., 0.0, 4,
                -30. * M_PI / 180., 26. * M_PI / 180., 0.2, 0.2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_point_cloud = lidar.Scan(cars);
    RenderRays(viewer, lidar.GetPosition(), scanned_point_cloud);
    RenderPointCloud(viewer, scanned_point_cloud, "scanned_point_cloud",
                     Color{1., 1., 0.});

    viewer->setCameraPosition(0, -35, 12, 0, 0, 1);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}