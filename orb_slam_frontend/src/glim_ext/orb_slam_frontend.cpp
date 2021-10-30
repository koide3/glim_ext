#include <glim_ext/orb_slam_frontend.hpp>

#include <System.h>
#include <ImuTypes.h>

namespace glim {

class OrbSLAMFrontend::Impl {
public:
  Impl() {
    const std::string voc_path = "";
    const std::string settings_path = "";
    system.reset(new ORB_SLAM3::System(voc_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true));
  }

private:
  std::unique_ptr<ORB_SLAM3::System> system;
};

OrbSLAMFrontend::OrbSLAMFrontend() {
  impl.reset(new Impl);
}

OrbSLAMFrontend::~OrbSLAMFrontend() {}

}  // namespace glim
