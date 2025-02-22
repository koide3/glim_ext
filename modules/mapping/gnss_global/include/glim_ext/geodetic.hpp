#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

Eigen::Vector3d ecef_to_wgs84(const Eigen::Vector3d& xyz);

Eigen::Vector3d wgs84_to_ecef(double lat, double lon, double alt);

Eigen::Isometry3d calc_T_ecef_nwz(const Eigen::Vector3d& ecef, double radius = 6378137);

double harversine(const Eigen::Vector2d& latlon1, const Eigen::Vector2d& latlon2);

}  // namespace gir