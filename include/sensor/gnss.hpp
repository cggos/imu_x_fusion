#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include <GeographicLib/LocalCartesian.hpp>

namespace cg {

struct GpsData {
  double timestamp;

  Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
  Eigen::Matrix3d cov;  // Covariance in m^2
};
using GpsDataPtr = std::shared_ptr<GpsData>;
using GpsDataConstPtr = std::shared_ptr<const GpsData>;

inline void lla2enu(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla, Eigen::Vector3d *point_enu) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), point_enu->data()[0], point_enu->data()[1],
                          point_enu->data()[2]);
}

inline void enu2lla(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_enu, Eigen::Vector3d *point_lla) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), point_lla->data()[0], point_lla->data()[1],
                          point_lla->data()[2]);
}

}  // namespace cg