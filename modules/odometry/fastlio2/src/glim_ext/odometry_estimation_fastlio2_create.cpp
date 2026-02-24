#include <glim_ext/odometry_estimation_fastlio2.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationFastLIO2Params params;
  return new glim::OdometryEstimationFastLIO2(params);
}
