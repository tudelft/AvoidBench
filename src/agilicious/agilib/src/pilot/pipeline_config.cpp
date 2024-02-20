#include "agilib/pilot/pipeline_config.hpp"

#include <ostream>

#include "agilib/base/parameter_base.hpp"
#include "agilib/utils/file_utils.hpp"

namespace agi {


void PipelineConfig::load(const Yaml& yaml, const std::string& directory) {
  estimator_cfg.loadIfUndefined(yaml["estimator"]);
  checkFile(directory, &estimator_cfg.file);

  sampler_cfg.loadIfUndefined(yaml["sampler"]);
  checkFile(directory, &sampler_cfg.file);

  if (!outer_controller_cfg.loadIfUndefined(yaml["outer_controller"]))
    outer_controller_cfg.loadIfUndefined(yaml["controller"]);
  checkFile(directory, &outer_controller_cfg.file);

  inner_controller_cfg.loadIfUndefined(yaml["inner_controller"]);
  checkFile(directory, &inner_controller_cfg.file);

  bridge_cfg.loadIfUndefined(yaml["bridge"]);
  checkFile(directory, &bridge_cfg.file);
}

std::ostream& operator<<(std::ostream& os, const PipelineConfig& config) {
  os << "Estimator:\n" << config.estimator_cfg;
  os << "Sampler:\n" << config.sampler_cfg;
  os << "Outer Controller:\n" << config.outer_controller_cfg;
  os << "Inner Controller:\n" << config.inner_controller_cfg;
  os << "Bridge:\n" << config.bridge_cfg;
  return os;
}

}  // namespace agi
