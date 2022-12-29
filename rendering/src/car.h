#ifndef CAR_H_
#define CAR_H_

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <string>

#include "common.h"

class Car {
  public:
    Car(const std::string& name, const Eigen::Vector3d& position,
        const Eigen::Vector3d& dimension, const Color& color,
        const double& opacity)
        : name_(name),
          position_(position),
          dimension_(dimension),
          color_(color),
          opacity_(opacity) {}

    // collision helper function
    bool inbetween(double point, double center, double range) {
        return (center - range <= point) && (center + range >= point);
    }

    bool checkCollision(Eigen::Vector3d point) {
        return (inbetween(point.x(), position_.x(), dimension_.x() / 2) &&
                inbetween(point.y(), position_.y(), dimension_.y() / 2) &&
                inbetween(point.z(), position_.z() + dimension_.z() / 3,
                          dimension_.z() / 3)) ||
               (inbetween(point.x(), position_.x(), dimension_.x() / 4) &&
                inbetween(point.y(), position_.y(), dimension_.y() / 2) &&
                inbetween(point.z(), position_.z() + dimension_.z() * 5 / 6,
                          dimension_.z() / 6));
    }

    void Render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
        // render bottom of car
        viewer->addCube(position_.x() - dimension_.x() / 2,
                        position_.x() + dimension_.x() / 2,
                        position_.y() - dimension_.y() / 2,
                        position_.y() + dimension_.y() / 2, position_.z(),
                        position_.z() + dimension_.z() * 2 / 3, color_.r,
                        color_.g, color_.b, name_ + "_body");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
            name_ + "_body");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity_,
            name_ + "_body");

        // render top of car
        viewer->addCube(position_.x() - dimension_.x() / 4,
                        position_.x() + dimension_.x() / 4,
                        position_.y() - dimension_.y() / 2,
                        position_.y() + dimension_.y() / 2,
                        position_.z() + dimension_.z() * 2 / 3,
                        position_.z() + dimension_.z(), color_.r, color_.g,
                        color_.b, name_ + "_top");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
            name_ + "_top");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity_,
            name_ + "_top");
    }

  private:
    std::string name_;
    Eigen::Vector3d position_;
    Eigen::Vector3d dimension_;
    Color color_;
    double opacity_;
};

#endif /* CAR */