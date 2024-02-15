#include "avoidlib/vision_envs/vec_vision_env_base.h"

namespace avoidlib {
template<typename EnvBaseName>
VecVisionEnvBase<EnvBaseName>::VecVisionEnvBase() {
  omp_set_num_threads(10);
}
template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::configEnv(const YAML::Node& cfg_node) {
  // initialization
  if (!cfg_node["unity"]["render"] || !cfg_node["simulation"]["seed"] ||
      !cfg_node["unity"]["scene_id"] || !cfg_node["simulation"]["num_envs"] ||
      !cfg_node["simulation"]["num_threads"]) {
    logger_.warn("Cannot load main configurations. Using default parameters.");
    unity_render_ = false;
    seed_ = 0;
    num_envs_ = 1;
    num_threads_ = 1;
    scene_id_ = 1;
  } else {
    //
    logger_.info("Load Unity configuration.");
    unity_render_ = cfg_node["unity"]["render"].as<bool>();
    scene_id_ = cfg_node["unity"]["scene_id"].as<SceneID>();
    spawn_trees_ = cfg_node["unity"]["spawn_trees"].as<bool>();
    spawn_objects_ = cfg_node["unity"]["spawn_objects"].as<bool>();
    bounding_box_ = cfg_node["unity"]["bounding_box"].as<std::vector<float>>();
    bounding_box_origin_ = cfg_node["unity"]["bounding_box_origin"].as<std::vector<float>>();
    min_object_scale_ = cfg_node["unity"]["min_object_scale"].as<std::vector<float>>();
    max_object_scale_ = cfg_node["unity"]["max_object_scale"].as<std::vector<float>>();
    if_get_pointcloud_ = cfg_node["unity"]["if_get_pointcloud"].as<bool>();
    range_ = cfg_node["unity"]["range"].as<std::vector<float>>();
    origin_ = cfg_node["unity"]["origin"].as<std::vector<float>>();
    res_ = cfg_node["unity"]["res"].as<float>();
    radius_origin_ = cfg_node["unity"]["radius_origin"].as<float>();
    radius_area_ = cfg_node["unity"]["radius_area"].as<float>();
    //
    logger_.info("Load Simulation configuration.");
    seed_ = cfg_node["simulation"]["seed"].as<int>();
    num_envs_ = cfg_node["simulation"]["num_envs"].as<int>();
    num_threads_ = cfg_node["simulation"]["num_threads"].as<int>();

    img_width_ = cfg_node["rgb_camera"]["width"].as<int>();
    img_height_ = cfg_node["rgb_camera"]["height"].as<int>();
    srand(seed_);
  }
    // set threads
  omp_set_num_threads(num_threads_);

    // create & setup environments
  for (int env_id = 0; env_id < num_envs_; env_id++) {
    envs_.push_back(std::make_unique<EnvBaseName>(cfg_node, env_id));
  }

    // set Unity
  if (unity_render_) {
    setUnity(unity_render_);
  }

  obs_dim_ = envs_[0]->getObsDim();
  goal_obs_dim_ = envs_[0]->getGoalObsDim();
  act_dim_ = envs_[0]->getActDim();
  seq_dim_ = envs_[0]->getSeqDim();
  state_dim_ = envs_[0]->getStateDim();
  rew_dim_ = envs_[0]->getRewDim();
  // img_width_ = envs_[0]->getImgWidth();
  // img_height_ = envs_[0]->getImgHeight();

  // generate reward names
  // compute it once to get reward names. actual value is not used
  envs_[0]->updateExtraInfo();
  for (auto& re : envs_[0]->extra_info_) {
    extra_info_names_.push_back(re.first);
  }
  logger_.info("%d vectorized enviromnets created. ", num_envs_);
  std::cout << "Vectorized Environment:\n"
            << "obs dim    =            [" << obs_dim_ << "]\n"
            << "act dim    =            [" << act_dim_ << "]\n"
            << "rew dim    =            [" << rew_dim_ << "]\n"
            << "einfo dim  =            [" << envs_[0]->extra_info_.size()
            << "]\n"
            << "img width  =            [" << img_width_ << "]\n"
            << "img height =            [" << img_height_ << "]\n"
            << "num_envs   =            [" << num_envs_ << "]\n"
            << "num_thread =            [" << num_threads_ << "]\n"
            << "seed       =            [" << seed_ << "]\n"
            << "scene_id   =            [" << scene_id_ << "]" << std::endl;
}

template<typename EnvBaseName>
VecVisionEnvBase<EnvBaseName>::~VecVisionEnvBase() {}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::reset(Ref<MatrixRowMajor<>> obs) {
  if (obs.rows() != num_envs_ || obs.cols() != goal_obs_dim_) {
    logger_.error(
      "Input matrix dimensions do not match with that of the environment.");
    return false;
  }

  receive_id_ = 0;
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->reset(obs.row(i));
  }

  return true;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::step(Ref<MatrixRowMajor<>> act,
                                   Ref<MatrixRowMajor<>> obs,
                                   Ref<MatrixRowMajor<>> reward,
                                   Ref<BoolVector<>> done,
                                   Ref<MatrixRowMajor<>> extra_info) {
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done, extra_info);
  }

  return true;
}

template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::close() {
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->close();
  }
}

template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::setSeed(const int seed) {
  int seed_inc = seed;
  for (int i = 0; i < num_envs_; i++) envs_[i]->setSeed(seed_inc++);
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::getObs(Ref<MatrixRowMajor<>> obs) {
  bool valid_obs = true;
  for (int i = 0; i < num_envs_; i++) valid_obs &= envs_[i]->getObs(obs.row(i));
  return valid_obs;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::getImage(Ref<ImgMatrixRowMajor<>> img,
                                       const bool rgb_image) {
  bool valid_img = true;
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    valid_img &= envs_[i]->getImage(img.row(i), rgb_image);
  }
  return valid_img;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::getDepthImage(
  Ref<DepthImgMatrixRowMajor<>> depth_img) {
  bool valid_img = true;
  for (int i = 0; i < num_envs_; i++) {
    valid_img &= envs_[i]->getDepthImage(depth_img.row(i));
  }
  return valid_img;
}

template<typename EnvBaseName>
size_t VecVisionEnvBase<EnvBaseName>::getEpisodeLength(void) {
  if (envs_.size() <= 0) {
    return 0;
  } else {
    return (size_t)envs_[0]->getMaxT() / envs_[0]->getSimTimeStep();
  }
}

template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::perAgentStep(int agent_id,
                                           Ref<MatrixRowMajor<>> act,
                                           Ref<MatrixRowMajor<>> obs,
                                           Ref<MatrixRowMajor<>> reward,
                                           Ref<BoolVector<>> done,
                                           Ref<MatrixRowMajor<>> extra_info) {
  // get individual rewards
  envs_[agent_id]->step(act.row(agent_id), obs.row(agent_id),
                        reward.row(agent_id));

  double terminal_reward = 0;
  done[agent_id] = envs_[agent_id]->isTerminalState(terminal_reward);

  envs_[agent_id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++)
    extra_info(agent_id, j) =
      envs_[agent_id]->extra_info_[extra_info_names_[j]];

  if (done[agent_id]) {
    envs_[agent_id]->reset(obs.row(agent_id));
    reward(agent_id, reward.cols() - 1) = terminal_reward;
  }
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::setUnity(bool render) {
  unity_render_ = render;
  if (!unity_render_ || unity_bridge_ptr_ != nullptr) {
    logger_.warn(
      "Unity render is False or Flightmare Bridge has been already created. "
      "Cannot set Unity.");
    return false;
  }
  // create unity bridge
  logger_.info("create unity bridge");
  unity_bridge_ptr_ = UnityBridge::getInstance();
  // add objects to Unity
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->addQuadrotorToUnity(unity_bridge_ptr_);
  }
  logger_.info("Flightmare Bridge created.");
  return true;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::setUnityFromPtr(std::shared_ptr<UnityBridge> bridge_ptr_) {
  if (unity_bridge_ptr_ != nullptr) {
    logger_.warn(
      "Flightmare Bridge has been already created. "
      "Cannot set Unity.");
    return false;
  }
  unity_bridge_ptr_ = bridge_ptr_;
  envs_[0]->setQuadFromPtr(bridge_ptr_);
  logger_.info("Flightmare Bridge created.");
  return true;
}

template<typename EnvBaseName>
std::shared_ptr<UnityBridge> VecVisionEnvBase<EnvBaseName>::getUnityPtr()
{
  // std::shared_ptr<Quadrotor> quad = unity_bridge_ptr_->getQuadrotor(0);
  return unity_bridge_ptr_;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::connectUnity(void) {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::initializeConnections() {
  setUnity(true);
}

template<typename EnvBaseName>
FrameID VecVisionEnvBase<EnvBaseName>::updateUnity(const FrameID frame_id) {
  if (unity_render_ && unity_ready_) {
    bool sent = unity_bridge_ptr_->getRender(frame_id, spawn_new_);
    spawn_new_ = false;
    if (!sent) {
      logger_.error("Message has not been sent successfully!!!!");
    }
    while(!unity_bridge_ptr_->handleOutput(frame_id)) {usleep(0.005*1e6);}
    return 1;
  } else {
    return 0;
  }
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::ifSceneChanged()
{
  return unity_bridge_ptr_->ifSceneChanged();
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::spawnObstacles(bool change_obs, int seed, float radius)
{
  if(change_obs)
  {
    if(spawn_trees_)
    {
      Tree_Message_t trees;
      trees.name = "trees";
      trees.bounding_area[0] = bounding_box_[0];
      trees.bounding_area[1] = bounding_box_[1];
      trees.bounding_origin[0] = bounding_box_origin_[0];
      trees.bounding_origin[1] = bounding_box_origin_[1];
      if(seed == -1)
        trees.seed = std::rand()%200;
      else
        trees.seed = seed;
      float rand = (std::rand()%200)/200.0f;
      if(radius == -1.0f)
        trees.radius = radius_origin_ + radius_area_ * rand;
      else
        trees.radius = radius;
      std::cout<<"trees radius: "<<trees.radius<<std::endl;
      unity_bridge_ptr_->placeTrees(trees);
    }
    if(spawn_objects_)
    {
      Object_Message_t objects;
      objects.name = "cylinder";
      objects.bounding_area[0] = bounding_box_[0];
      objects.bounding_area[1] = bounding_box_[1];
      objects.bounding_origin[0] = bounding_box_origin_[0];
      objects.bounding_origin[1] = bounding_box_origin_[1];
      objects.scale_min = min_object_scale_;
      objects.scale_max = max_object_scale_;
      if(seed == -1)
        objects.seed = std::rand()%200;
      else
        objects.seed = seed;
      float rand = (std::rand()%200) / 200.0f;
      if(radius == -1.0f)
      {
        objects.radius = radius_origin_ + radius_area_ * rand;
        objects.opacity = (std::rand()%200) / 200.0f;
      }
      else
      {
        objects.radius = radius;
        objects.opacity = 0.5f;
      }
      unity_bridge_ptr_->placeObjects(objects);
    }
  }

  spawn_new_ = true;

  return true;
}
template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::getPointClouds(const std::string curr_data_dir, int id, bool save_pc)
{
  if (save_pc)
  {
    // save point clouds
    save_pc_success_ = false;
    PointCloudMessage_t pc_msg;
    pc_msg.range = range_;
    pc_msg.origin = origin_;
    if(curr_data_dir != "")
      pc_msg.path = curr_data_dir;
    pc_msg.file_name = "pointclouds" + std::to_string(id);
    unity_bridge_ptr_->getPointCloud(pc_msg);
    save_pc_success_ = true;
  }
  return true;
}

template<typename EnvBaseName>
bool VecVisionEnvBase<EnvBaseName>::readPointClouds(int id)
{
  read_pc_success_ = false;
  // read the point clouds and save it to kd-tree
  std::string pc_path = getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/point_clouds_data/") + 
                        "pointclouds" + std::to_string(id) + std::string(".ply");
  env_ptr_->readPointCloud(pc_path);
  double traversability = env_ptr_->getTraversability();
  std::cout<<"traversability: "<<traversability<<std::endl;
  for (int i = 0; i < num_envs_; i++) {
    envs_[i]->setPointClouds(env_ptr_);
    envs_[i]->setTraversability(traversability);
  }
  read_pc_success_ = true;
  return true;
}

template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::isTerminalState(
  Ref<BoolVector<>> terminal_state) {}

template<typename EnvBaseName>
void VecVisionEnvBase<EnvBaseName>::disconnectUnity(void) {
  if (unity_bridge_ptr_ != nullptr) {
    unity_bridge_ptr_->disconnectUnity();
    unity_ready_ = false;
  } else {
    logger_.warn("Flightmare Unity Bridge is not initialized.");
  }
}

template class VecVisionEnvBase<AvoidVisionEnv>;

}