#pragma once

#include <ostream>

#include "agilib/utils/filesystem.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi {

struct ModuleConfig {
  std::string type;
  fs::path file;

  bool loadIfUndefined(const Yaml& yaml);

  friend std::ostream& operator<<(std::ostream& os, const ModuleConfig& config);
};
}  // namespace agi