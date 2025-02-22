#include <glim_ext/gravity_estimator.hpp>

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GravityEstimatorModule();
}