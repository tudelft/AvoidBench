#include "agilib/utils/module_config.hpp"

namespace agi {

std::ostream& operator<<(std::ostream& os, const ModuleConfig& config) {
  return os << "Type: " << config.type << "\nFile: " << config.file << '\n';
}

bool ModuleConfig::loadIfUndefined(const Yaml& yaml) {
  if (type.empty()) yaml["type"].getIfDefined(type);

  if (file.empty()) yaml["file"].getIfDefined(file);

  return !type.empty() && !file.empty();
}

}  // namespace agi
