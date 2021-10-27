#pragma once

#include <memory>

namespace glim {

class ScanContextLoopDetector {
public:
  ScanContextLoopDetector();
  ~ScanContextLoopDetector();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}  // namespace glim
