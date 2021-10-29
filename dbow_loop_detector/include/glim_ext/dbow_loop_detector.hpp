#pragma once

#include <memory>

namespace glim {

class DBoWLoopDetector {
public:
  DBoWLoopDetector();
  ~DBoWLoopDetector();

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}  // namespace glim
