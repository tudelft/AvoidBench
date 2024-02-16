#pragma once

#include <ostream>

#include "agilib/utils/filesystem.hpp"
#include "agilib/utils/module_config.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi {

struct PipelineConfig {
  ModuleConfig estimator_cfg;
  ModuleConfig sampler_cfg;
  ModuleConfig outer_controller_cfg;
  ModuleConfig inner_controller_cfg;
  ModuleConfig bridge_cfg;

  void load(const Yaml& yaml, const std::string& directory);

  friend std::ostream& operator<<(std::ostream& os,
                                  const PipelineConfig& config);
};


}  // namespace agi
