#include <glim_ext/imu_prediction_module.hpp>

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::IMUPredictionModule();
}