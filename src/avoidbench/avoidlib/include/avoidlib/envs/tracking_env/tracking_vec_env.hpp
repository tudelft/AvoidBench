#pragma once

// yaml cpp
#include <yaml-cpp/yaml.h>

#include "avoidlib/envs/tracking_env/tracking_env.hpp"
#include "avoidlib/envs/vec_env_base.hpp"

namespace avoidlib {

template<typename EnvBaseName>
class TrackingVecEnv : public VecEnvBase<EnvBaseName> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TrackingVecEnv();
  TrackingVecEnv(const std::string& cfg, const bool from_file = true);
  TrackingVecEnv(const YAML::Node& cfg_node);
  ~TrackingVecEnv();

  using VecEnvBase<EnvBaseName>::configEnv;

  bool reset(Ref<MatrixRowMajor<>> obs) override;
  bool reset(Ref<MatrixRowMajor<>> obs, bool random);
  bool reset(Ref<MatrixRowMajor<>> obs, Ref<MatrixRowMajor<>> state, Ref<MatrixRowMajor<>> omega);
  bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info) override;


  bool getQuadAct(Ref<MatrixRowMajor<>> quadact);
  bool getQuadState(Ref<MatrixRowMajor<>> quadstate);
  inline std::vector<std::string> getRewardNames(void) {
    return this->envs_[0]->getRewardNames();
  };

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

}  // namespace avoidlib
