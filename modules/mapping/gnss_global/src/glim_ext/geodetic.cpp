#include <glim_ext/geodetic.hpp>

namespace glim {

/**
 * Code taken from ethz-asl/geodetic_utils (BSD3 https://github.com/ethz-asl/geodetic_utils/blob/master/geodetic_utils/LICENSE)
 * https://github.com/ethz-asl/geodetic_utils/blob/master/geodetic_utils/include/geodetic_utils/geodetic_conv.hpp
 */

// Geodetic system parameters
static const double kSemimajorAxis = 6378137;
static const double kSemiminorAxis = 6356752.3142;
static const double kFirstEccentricitySquared = 6.69437999014 * 0.001;
static const double kSecondEccentricitySquared = 6.73949674228 * 0.001;
static const double kFlattening = 1 / 298.257223563;

Eigen::Vector3d ecef_to_wgs84(const Eigen::Vector3d& xyz) {
  // Convert ECEF coordinates to geodetic coordinates.
  // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
  // to geodetic coordinates," IEEE Transactions on Aerospace and
  // Electronic Systems, vol. 30, pp. 957-961, 1994.

  const double x = xyz.x();
  const double y = xyz.y();
  const double z = xyz.z();

  double r = sqrt(x * x + y * y);
  double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
  double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
  double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
  double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
  double S = cbrt(1 + C + sqrt(C * C + 2 * C));
  double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
  double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
  double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) +
               sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
  double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
  double V = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
  double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);

  const double alt = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
  const double lat = atan((z + kSecondEccentricitySquared * Z_0) / r) * 180.0 / M_PI;
  const double lon = atan2(y, x) * 180.0 / M_PI;

  return {lat, lon, alt};
}

Eigen::Vector3d wgs84_to_ecef(double lat, double lon, double alt) {
  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;
  double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
  const double x = (kSemimajorAxis / xi + alt) * cos(lat_rad) * cos(lon_rad);
  const double y = (kSemimajorAxis / xi + alt) * cos(lat_rad) * sin(lon_rad);
  const double z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + alt) * sin(lat_rad);

  return {x, y, z};
}

Eigen::Isometry3d calc_T_ecef_nwz(const Eigen::Vector3d& ecef, double radius) {
  const Eigen::Vector3d z = ecef.normalized();
  const Eigen::Vector3d to_north = (Eigen::Vector3d::UnitZ() * radius - ecef).normalized();
  const Eigen::Vector3d x = (to_north - to_north.dot(z) * z).normalized();
  const Eigen::Vector3d y = z.cross(x);

  Eigen::Isometry3d T_ecef_nwz = Eigen::Isometry3d::Identity();
  T_ecef_nwz.linear().col(0) = x;
  T_ecef_nwz.linear().col(1) = y;
  T_ecef_nwz.linear().col(2) = z;
  T_ecef_nwz.translation() = ecef;

  return T_ecef_nwz;
}

double harversine(const Eigen::Vector2d& latlon1, const Eigen::Vector2d& latlon2) {
  const double lat1 = latlon1[0];
  const double lon1 = latlon1[1];
  const double lat2 = latlon2[0];
  const double lon2 = latlon2[1];

  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;

  const double a = std::pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * std::pow(sin(dlon / 2), 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return kSemimajorAxis * c;
}

}  // namespace gir
