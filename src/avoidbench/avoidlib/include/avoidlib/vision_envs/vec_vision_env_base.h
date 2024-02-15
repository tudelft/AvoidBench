#pragma once

// std
#include <memory>

// openmp
#include <omp.h>

// avoidlib
#include "avoidlib/bridges/unity_bridge.hpp"
#include "avoidlib/common/logger.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/vision_envs/vision_env_base.h"
#include "avoidlib/vision_envs/avoid_vision_envs/avoid_vision_envs.h"

namespace avoidlib {

template<typename EnvBaseName>
class VecVisionEnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VecVisionEnvBase();
  virtual ~VecVisionEnvBase() = 0;

  virtual void configEnv(const YAML::Node& cfg_node);
  virtual bool reset(Ref<MatrixRowMajor<>> obs);
  virtual bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                    Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info);
  virtual void close();
  // public set functions
  void setSeed(const int seed);

  bool getObs(Ref<MatrixRowMajor<>> obs);
  bool getImage(Ref<ImgMatrixRowMajor<>> img, const bool rgb = false);
  bool getDepthImage(Ref<DepthImgMatrixRowMajor<>> img);

  size_t getEpisodeLength(void);

  // - auxiliary functions
  void isTerminalState(Ref<BoolVector<>> terminal_state);

  // flightmare (visualization)
  bool setUnity(const bool render);
  bool setUnityFromPtr(std::shared_ptr<UnityBridge> bridge_ptr_);
  std::shared_ptr<UnityBridge> getUnityPtr();
  bool connectUnity();
  bool initializeConnections();
  void disconnectUnity();
  FrameID updateUnity(const FrameID frame_id);
  bool spawnObstacles(bool change_obs, int seed=-1, float radius=-1.0f);
  bool ifSceneChanged();
  bool getPointClouds(const std::string curr_data_dir, int id, bool save_pc);
  bool readPointClouds(int id);
  bool getSavingState() { return save_pc_success_; };
  bool getReadingState() { return read_pc_success_; };
    // public functions
  inline int getSeed(void) const { return seed_; };
  inline SceneID getSceneID(void) const { return scene_id_; };
  inline bool getUnityRender(void) const { return unity_render_; };
  inline int getObsDim(void) const { return obs_dim_; };
  inline int getGoalObsDim(void) const { return goal_obs_dim_; };
  inline int getActDim(void) const { return act_dim_; };
  inline int getSeqDim(void) const { return seq_dim_; };
  inline int getStateDim(void) const { return state_dim_; };
  inline int getRewDim(void) const { return rew_dim_; };
  inline int getImgHeight(void) const { return img_height_; };
  inline int getImgWidth(void) const { return img_width_; };
  inline int getExtraInfoDim(void) const { return extra_info_names_.size(); };
  inline int getNumOfEnvs(void) const { return envs_.size(); };
  inline std::vector<std::string>& getExtraInfoNames() {
    return extra_info_names_;
  };

 protected:
  // step every environment
  virtual void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                            Ref<MatrixRowMajor<>> obs,
                            Ref<MatrixRowMajor<>> reward_units,
                            Ref<BoolVector<>> done,
                            Ref<MatrixRowMajor<>> extra_info);
  // create objects
  Logger logger_{"VecVisionEnvBase"};

  std::vector<std::unique_ptr<EnvBaseName>> envs_;
  std::vector<std::string> extra_info_names_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  std::shared_ptr<Environment> env_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliar variables
  int seed_, num_envs_, obs_dim_, goal_obs_dim_, seq_dim_, act_dim_, state_dim_, rew_dim_, num_threads_;
  std::vector<float> bounding_box_, bounding_box_origin_;
  std::vector<float> min_object_scale_, max_object_scale_;
  bool if_get_pointcloud_;
  std::vector<float> range_, origin_;
  float res_;
  std::string file_name_;
  // unity set up
  bool spawn_trees_, spawn_objects_;
  bool spawn_new_ {false};
  float radius_area_, radius_origin_;

  int img_width_, img_height_;
  Matrix<> obs_dummy_;
  bool save_pc_success_{false};
  bool read_pc_success_{false};
};
}