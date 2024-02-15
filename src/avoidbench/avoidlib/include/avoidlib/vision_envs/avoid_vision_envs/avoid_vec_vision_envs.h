#pragma once

// yaml cpp
#include <yaml-cpp/yaml.h>

#include "avoidlib/vision_envs/avoid_vision_envs/avoid_vision_envs.h"
#include "avoidlib/vision_envs/vec_vision_env_base.h"

namespace avoidlib {

template<typename EnvBaseName>
class AvoidVecVisionEnv : public VecVisionEnvBase<EnvBaseName> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AvoidVecVisionEnv();
  AvoidVecVisionEnv(const std::string& cfg, const bool from_file = true);
  AvoidVecVisionEnv(const YAML::Node& cfg_node);
  ~AvoidVecVisionEnv();

  using VecVisionEnvBase<EnvBaseName>::configEnv;

  bool reset(Ref<MatrixRowMajor<>> obs) override;
  bool reset(Ref<MatrixRowMajor<>> obs, bool random);
  // bool reset(Ref<MatrixRowMajor<>> obs, Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> omega);
  bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info) override;

  bool getQuadAct(Ref<MatrixRowMajor<>> quadact);
  bool getQuadState(Ref<MatrixRowMajor<>> quadstate);
  inline std::vector<std::string> getRewardNames(void) {
    return this->envs_[0]->getRewardNames();
  };
  bool resetRewCoeff();

private:
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                    Ref<MatrixRowMajor<>> obs,
                    Ref<MatrixRowMajor<>> reward_units, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info) override;

  // yaml configurations
  bool random_reset_;
  //
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;
};

}