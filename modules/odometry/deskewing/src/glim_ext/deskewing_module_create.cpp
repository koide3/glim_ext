#include <glim_ext/deskewing_module.hpp>

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::DeskewingModule();
}