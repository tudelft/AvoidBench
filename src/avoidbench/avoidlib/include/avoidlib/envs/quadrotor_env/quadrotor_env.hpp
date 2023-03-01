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

namespace quadenv {

enum Ctl : int {
  //
  kNQuadState = 32,

  // observations
  kObs = 0,
  kNObs = 21,
  kNState = 21,
  kNMotor = 4,

  // control actions
  kAct = 0,
  kNAct = 4
};
}  // namespace quadenv

class QuadrotorEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadrotorEnv();
  QuadrotorEnv(const std::string &cfg_path, const int env_id);
  QuadrotorEnv(const YAML::Node &cfg_node, const int env_id);
  ~QuadrotorEnv();

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
                                  const QuadrotorEnv &quad_env);

  inline std::vector<std::string> getRewardNames() { return reward_names_; }

  std::unordered_map<std::string, float> extra_info_;

 private:
  void init();
  int env_id_;
  bool configCamera(const YAML::Node &cfg, const std::shared_ptr<RGBCamera>);
  // quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  QuadState quad_state_;
  QuadState pre_quad_state_;
  Command cmd_;
  Logger logger_{"QaudrotorEnv"};


  // Define reward for training
  double pos_coeff_, vert_coeff_, ori_coeff_, lin_vel_coeff_, ang_vel_coeff_, smooth_coeff_;
  double vel_dir_coeff_, time_coeff_;
  // observations and actions (for RL)
  Vector<quadenv::kNObs> pi_obs_;
  Vector<quadenv::kNAct> pi_act_;

  // reward function design (for model-free reinforcement learning)
  Vector<3> goal_pos_, next_goal_pos, global_goal_pos_;
  Vector<3> goal_vel_;
  Vector<3> start;
  double vel_lin_, vel_ang_;
  double process;
  double s_stamp, e_stamp;

  // action and observation normalization (for learning)
  Vector<quadenv::kNAct> act_mean_;
  Vector<quadenv::kNAct> act_std_;
  Vector<quadenv::kNObs> obs_mean_ = Vector<quadenv::kNObs>::Zero();
  Vector<quadenv::kNObs> obs_std_ = Vector<quadenv::kNObs>::Ones();

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
