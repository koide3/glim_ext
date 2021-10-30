#pragma once

#include <memory>

namespace glim {

class OrbSLAMFrontend {
public:
  OrbSLAMFrontend();
  ~OrbSLAMFrontend();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};
}  // namespace glim