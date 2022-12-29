#ifndef LIDAR_H
#define LIDAR_H

#include <pcl/common/common.h>

#include <Eigen/Core>
#include <chrono>
#include <ctime>
#include <iostream>
#include <vector>

#include "car.h"

class Ray {
  public:
    Ray(const Eigen::Vector3d& origin, const double& horizontal_angle,
        const double& vertical_angle, const double& resolution)
        : origin_(origin),
          resolution_(resolution),
          direction_(resolution * cos(vertical_angle) * cos(horizontal_angle),
                     resolution * cos(vertical_angle) * sin(horizontal_angle),
                     resolution * sin(vertical_angle)) {}

    void Cast(const std::vector<Car>& cars, double min_distance,
              double max_distance, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
              double ground_slope, double sd_error) {
        // reset ray
        Eigen::Vector3d cast_position = origin_;
        double cast_distance = 0.;

        bool collision = false;

        while (!collision && cast_distance < max_distance) {
            cast_position = cast_position + direction_;
            cast_distance += resolution_;

            // check if there is any collisions with ground slope
            collision =
                (cast_position.z() <= cast_position.x() * tan(ground_slope));

            // check if there is any collisions with cars
            if (!collision && cast_distance < max_distance) {
                for (Car car : cars) {
                    collision |= car.checkCollision(cast_position);
                    if (collision) break;
                }
            }
        }

        if ((cast_distance >= min_distance) &&
            (cast_distance <= max_distance)) {
            // add noise based on standard deviation error
            double rx = ((double)rand() / (RAND_MAX));
            double ry = ((double)rand() / (RAND_MAX));
            double rz = ((double)rand() / (RAND_MAX));
            cloud->points.push_back(
                pcl::PointXYZ(cast_position.x() + rx * sd_error,
                              cast_position.y() + ry * sd_error,
                              cast_position.z() + rz * sd_error));
        }
    }

  private:
    Eigen::Vector3d origin_;
    double resolution_;
    Eigen::Vector3d direction_;

    // parameters:
    // setOrigin: the starting position_ from where the ray is cast
    // horizontalAngle: the angle of direction the ray travels on the xy plane
    // verticalAngle: the angle of direction between xy plane and ray
    // 				  for example 0 radians is along xy plane and
    // M_PI/2 radians is stright up resolution_: the magnitude of the ray's
    // step, used for ray casting, the smaller the more accurate but the more
    // expensive
};

class Lidar {
  public:
    Lidar(const Eigen::Vector3d position, const double& min_distance,
          const double& max_distance, const double& resolution,
          const double& ground_slope, const int& num_layers,
          const double& min_verticle_angle, const double& verticle_angle_range,
          const double& ray_resolution, const double& sd_error)
        : position_(position),
          min_distance_(min_distance),
          max_distance_(max_distance),
          resolution_(resolution),
          ground_slope_(ground_slope),
          num_layers_(num_layers),
          min_verticle_angle_(min_verticle_angle),
          verticle_angle_range_(verticle_angle_range),
          ray_resolution_{ray_resolution},
          sd_error_(sd_error) {
        double angle_increment = verticle_angle_range_ / num_layers_;
        double verticle_angle = min_verticle_angle_;
        for (double horizontal_angle = 0; horizontal_angle < 2 * M_PI;
             horizontal_angle += resolution_) {
            Ray ray(position_, horizontal_angle, verticle_angle,
                    ray_resolution_);
            rays_.push_back(ray);
        }

        for (int i = 0; i < num_layers_ - 1; ++i) {
            verticle_angle += angle_increment;
            for (double horizontal_angle = 0; horizontal_angle < 2 * M_PI;
                 horizontal_angle += resolution_) {
                Ray ray(position_, horizontal_angle, verticle_angle,
                        ray_resolution_);
                rays_.push_back(ray);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Scan(const std::vector<Car> cars) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        auto start_time = std::chrono::steady_clock::now();
        for (Ray ray : rays_) {
            ray.Cast(cars, min_distance_, max_distance_, cloud, ground_slope_,
                     sd_error_);
        }
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                  start_time);
        std::cout << "ray casting took " << elapsed_time.count()
                  << " milliseconds" << std::endl;
        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    Eigen::Vector3d GetPosition() { return position_; }

  private:
    std::vector<Ray> rays_;
    Eigen::Vector3d position_;
    double min_distance_;
    double max_distance_;
    double resolution_;
    double ground_slope_;
    int num_layers_;
    double min_verticle_angle_;
    double verticle_angle_range_;
    double ray_resolution_;
    double sd_error_;
};

#endif