#include <iostream>
#include <spdlog/spdlog.h>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

/// @brief Callback sink that calls a user-defined callback function for each log message.
/// @note  This is identical to spdlog::sinks::callback_sink, just for backwards compatibility with older spdlog versions.
template <typename Mutex>
class callback_sink final : public spdlog::sinks::base_sink<Mutex> {
public:
  using Callback = std::function<void(const spdlog::details::log_msg& msg)>;
  explicit callback_sink(const Callback& callback) : callback_{callback} {}

protected:
  void sink_it_(const spdlog::details::log_msg& msg) override { callback_(msg); }
  void flush_() override {}

private:
  Callback callback_;
};

using callback_sink_mt = callback_sink<std::mutex>;

/// @brief A demo extension module that hooks into all loggers and prints log messages to the console.
/// @note  This module is assumed to be loaded after all other modules, so that it can hook into all loggers.
class LoggerHookDemo : public ExtensionModule {
public:
  LoggerHookDemo() : logger(create_module_logger("loghook")) {
    // Message callback sink to be added to all loggers
    auto sink = std::make_shared<callback_sink_mt>([this](const auto& msg) { msg_callback(msg); });

    // Add the callback sink to all existing loggers
    spdlog::apply_all([this, sink](std::shared_ptr<spdlog::logger> logger) {
      if (logger->name() == "loghook") {
        return;
      }

      this->logger->info("Adding callback sink to logger: {}", logger->name());
      logger->sinks().push_back(sink);
    });
  }

  ~LoggerHookDemo() {}

  /// @brief Log message callback function
  /// @param msg  Log message
  void msg_callback(const spdlog::details::log_msg& msg) {  //
    const std::string_view logger_name(msg.logger_name.data(), msg.logger_name.size());
    const std::string level = fmt::format(spdlog::level::to_string_view(msg.level));
    const std::string_view payload(msg.payload.data(), msg.payload.size());
    std::cout << "[LoggerHookDemo] " << logger_name << " [" << level << "] " << payload << std::endl;
  }

private:
  std::shared_ptr<spdlog::logger> logger;  // Logger
};
}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::LoggerHookDemo();
}