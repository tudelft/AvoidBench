#pragma once

// std lib
#include <stdlib.h>

#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// avoidlib
#include "avoidlib/bridges/unity_bridge.hpp"
#include "avoidlib/common/command.hpp"
#include "avoidlib/common/logger.hpp"
#include "avoidlib/common/quad_state.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/common/utils.hpp"
#include "avoidlib/envs/env_base.hpp"
#include "avoidlib/objects/quadrotor.hpp"
#include "avoidlib/sensors/rgb_camera.hpp"

namespace avoidlib {

namespace trackenv {

enum Ctl : int {
  //
  kNQuadState = 29,

  // observations
  kObs = 0,
  kNObs = 16,
  kNState = 14,
  kNMotor = 4,

  // control actions
  kAct = 0,
  kNAct = 4
};
}  // namespace quadenv

class TrackingEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TrackingEnv();
  TrackingEnv(const std::string &cfg_path, const int env_id);
  TrackingEnv(const YAML::Node &cfg_node, const int env_id);
  ~TrackingEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs) override;
  bool reset(Ref<Vector<>> obs, bool random);
  bool reset(Ref<Vector<>> obs, Ref<Vector<>> state);
  bool reset(Ref<Vector<>> obs, Ref<Vector<>> state, Ref<Vector<>> omega);

  bool step(const Ref<Vector<>> act, Ref<Vector<>> obs,
            Ref<Vector<>> reward) override;

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getImage(Ref<ImgVector<>> img, const bool rgb = true) override;
  bool getDepthImage(Ref<DepthImgVector<>> img) override;

  // get quadrotor states
  bool getQuadAct(Ref<Vector<>> act) const;
  bool getQuadState(Ref<Vector<>> state) const;

  // - auxiliar functions
  bool isTerminalState(double &reward) override;
  bool addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) override;

  friend std::ostream &operator<<(std::ostream &os,
                                  const TrackingEnv &quad_env);

  inline std::vector<std::string> getRewardNames() { return reward_names_; }

  std::unordered_map<std::string, float> extra_info_;

 private:
  void init();
  int env_id_;
  bool configCamera(const YAML::Node &cfg, const std::shared_ptr<RGBCamera>);
  // quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"QaudrotorEnv"};


  // Define reward for training
  double pos_coeff_, ori_coeff_, lin_vel_coeff_, ang_vel_coeff_;

  // observations and actions (for RL)
  Vector<trackenv::kNObs> pi_obs_;
  Vector<trackenv::kNAct> pi_act_;

  // reward function design (for model-free reinforcement learning)
  Vector<3> init_target_pos_;

  // action and observation normalization (for learning)
  Vector<trackenv::kNAct> act_mean_;
  Vector<trackenv::kNAct> act_std_;
  Vector<trackenv::kNObs> obs_mean_ = Vector<trackenv::kNObs>::Zero();
  Vector<trackenv::kNObs> obs_std_ = Vector<trackenv::kNObs>::Ones();

  // robot vision
  std::shared_ptr<RGBCamera> rgb_camera_;
  cv::Mat rgb_img_, gray_img_;
  cv::Mat depth_img_;

  // auxiliary variables
  int rotor_ctrl_{true};
  bool use_camera_{false};
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;
  Matrix<3, 2> world_box_;
};

}  // namespace avoidlib
