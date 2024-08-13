#pragma once

#include <iostream>
#include <glim/util/config.hpp>

namespace glim {

class GlobalConfigExt : public Config {
private:
  GlobalConfigExt(const std::string& global_config_path) : Config(global_config_path) {}
  virtual ~GlobalConfigExt() override {}

public:
  static GlobalConfigExt* instance(std::string config_path = "") {
    if (inst == nullptr) {
      const auto config_ext_path = GlobalConfig::instance()->param<std::string>("global", "config_ext");
      if (!config_ext_path) {
        std::cerr << "warning: failed to find config_ext path!!" << std::endl;
      } else {
        config_path = *config_ext_path;
      }

      inst = new GlobalConfigExt(config_path + "/config_ext.json");
      inst->override_param("global_ext", "config_path", config_path);
    }
    return inst;
  }

  static std::string get_data_path() {
    auto config = instance();
    const std::string config_path = config->param<std::string>("global_ext", "config_path", ".");
    const std::string data_path = config->param<std::string>("global_ext", "data_path", "../data");
    return config_path + "/" + data_path;
  }

  static std::string get_config_path(const std::string& config_name) {
    const auto global_config = GlobalConfig::instance();
    if (global_config->param<std::string>("global", config_name)) {
      return global_config->get_config_path(config_name);
    }

    const auto config = instance();
    const std::string directory = config->param<std::string>("global_ext", "config_path", ".");
    const std::string filename = config->param<std::string>("global_ext", config_name, config_name + ".json");
    return directory + "/" + filename;
  }

  static GlobalConfigExt* inst;
};
}  // namespace glim