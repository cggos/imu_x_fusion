#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

namespace cg {

struct ImuData {
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};
using ImuDataPtr = std::shared_ptr<ImuData>;
using ImuDataConstPtr = std::shared_ptr<const ImuData>;

struct GpsData {
    double timestamp;

    Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
    Eigen::Matrix3d cov;  // Covariance in m^2
};
using GpsDataPtr = std::shared_ptr<GpsData>;
using GpsDataConstPtr = std::shared_ptr<const GpsData>;

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}

inline void lla2enu(const Eigen::Vector3d &init_lla,
                    const Eigen::Vector3d &point_lla,
                    Eigen::Vector3d *point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}

inline void enu2lla(const Eigen::Vector3d &init_lla,
                    const Eigen::Vector3d &point_enu,
                    Eigen::Vector3d *point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
}

Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field);

Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q);
Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q);

}  // namespace cg
