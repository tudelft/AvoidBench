#pragma once

// standard library
#include <unistd.h>

#include <cmath>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

// yaml
#include <yaml-cpp/yaml.h>

// avoidlib
#include "avoidlib/bridges/unity_bridge.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/pointclouds/environment.h"

namespace avoidlib {
class VisionEnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisionEnvBase();
  virtual ~VisionEnvBase() = 0;
  virtual bool reset(Ref<Vector<>> obs) = 0;
  virtual bool step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                    Ref<Vector<>> reward) = 0;
  virtual bool getObs(Ref<Vector<>> obs) = 0;
  virtual bool getImage(Ref<ImgVector<>> img, const bool rgb = true);
  virtual bool getDepthImage(Ref<DepthImgVector<>> img);
  virtual void close();
  virtual void render();
  virtual void updateExtraInfo();
  virtual bool isTerminalState(double &reward);
  virtual bool addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge);
  virtual bool setPointClouds(const std::shared_ptr<Environment> env_ptr);
  virtual void setTraversability(double traversability);
  virtual void setQuadFromPtr(const std::shared_ptr<UnityBridge> bridge);
  inline void setSeed(const int seed) { std::srand(seed); };
  inline int getObsDim() { return obs_dim_; };
  inline int getGoalObsDim() { return goal_obs_dim_; };
  inline int getActDim() { return act_dim_; };
  inline int getSeqDim() { return seq_dim_; };
  inline int getRewDim() { return rew_dim_; };
  inline int getStateDim() {return state_dim_; };
  inline int getImgWidth() { return img_width_; };
  inline int getImgHeight() { return img_height_; };
  inline double getSimTimeStep() { return sim_dt_; };
  inline int getExtraInfoDim() { return 0; };
  inline double getMaxT() { return max_t_; };

 protected:
  // observation and action dimenstions (for Reinforcement learning)
  int obs_dim_;
  int goal_obs_dim_;
  int act_dim_;
  int seq_dim_;
  int state_dim_;
  //
  int rew_dim_;
  int img_width_;
  int img_height_;

  // control time step
  double sim_dt_;
  double max_t_;
  int action_mode_;
  // random variable generator
  std::normal_distribution<double> norm_dist_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_dist_{-1.0, 1.0};
  std::random_device rd_;
  std::mt19937 random_gen_{rd_()};
};
}