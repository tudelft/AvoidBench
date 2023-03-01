#include "avoidlib/vision_envs/avoid_vision_envs/avoid_vision_envs.h"

namespace avoidlib {

AvoidVisionEnv::AvoidVisionEnv()
  : AvoidVisionEnv(getenv("AVOIDBENCH_PATH") +
                   std::string("/flightpy/configs/control/config_gazeo.yaml"),
                 0) {}
  
AvoidVisionEnv::AvoidVisionEnv(const std::string &cfg_path, const int env_id)
  : VisionEnvBase() {
  // check if configuration file exist
  if (!(file_exists(cfg_path))) {
    logger_.error("Configuration file %s does not exists.", cfg_path);
  }
  // load configuration file
  cfg_ = YAML::LoadFile(cfg_path);
  //
  init();
  env_id_ = env_id;
}

AvoidVisionEnv::AvoidVisionEnv(const YAML::Node &cfg_node, const int env_id)
  : VisionEnvBase(), cfg_(cfg_node) {
  //
  init();
  env_id_ = env_id;
}

void AvoidVisionEnv::init() {
  obs_dim_ = avoidenv::kNObs + avoidenv::kNLatent;
  goal_obs_dim_ = avoidenv::kNObs;
  act_dim_ = avoidenv::kNAct * avoidenv::kNSeq;
  seq_dim_ = avoidenv::kNSeq;
  state_dim_ = avoidenv::kNState;
  rew_dim_ = 0;
  round = 0;

  world_box_ << -10, 10, 0, 32, 0.0, 4.5;

  // load parameters
  loadParam(cfg_);
  if(is_training)
  {
    quad_ptr_ = std::make_shared<Quadrotor>();
    // define input and output dimension for the environment
    if (!quad_ptr_->setWorldBox(world_box_)) {
      logger_.error("cannot set wolrd box");
    };

    // add camera
    if (use_camera_) {
      rgb_camera_ = std::make_shared<RGBCamera>();
      if (!configCamera(cfg_, rgb_camera_)) {
        logger_.error(
          "Cannot config RGB Camera. Something wrong with the config file");
      };

      quad_ptr_->addRGBCamera(rgb_camera_);
      //
      img_width_ = rgb_camera_->getWidth();
      img_height_ = rgb_camera_->getHeight();
      rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                                CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
      depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    }
  }

}

AvoidVisionEnv::~AvoidVisionEnv() {}

void AvoidVisionEnv::setQuadFromPtr(const std::shared_ptr<UnityBridge> bridge)
{
  quad_ptr_ = bridge->getQuadrotor(0);
  if (use_camera_)
  {
    rgb_camera_ = quad_ptr_->getCameras()[0];
    img_width_ = rgb_camera_->getWidth();
    img_height_ = rgb_camera_->getHeight();
    rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                              CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
    depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
  }
}

bool AvoidVisionEnv::reset(Ref<Vector<>> obs) {
  // reset position
  quad_state_.setZero();
  quad_state_.x(QS::POSX) = uniform_dist_(random_gen_);
  quad_state_.x(QS::POSY) = uniform_dist_(random_gen_) + 1.0;
  quad_state_.x(QS::POSZ) = 2.0;
  if(action_mode_ ==0)
  {
    quad_state_.x(QS::VELX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELZ) = uniform_dist_(random_gen_);
  }
  else {
    quad_state_.x(QS::VELX) = 0.0;
    quad_state_.x(QS::VELY) = 0.0;
    quad_state_.x(QS::VELZ) = 0.0;
  }

  Vector<3> euler;
  euler.x() = 0.0;
  euler.y() = 0.0;
  euler.z() = 0.0;
  Quaternion quat;
  EularToquaternion(quat, euler);
  quad_state_.x(QS::ATTW) = quat.w();
  quad_state_.x(QS::ATTX) = quat.x();
  quad_state_.x(QS::ATTY) = quat.y();
  quad_state_.x(QS::ATTZ) = quat.z();
  pre_quad_state_ = quad_state_;
  // reset quadrotor with random states
  quad_ptr_->reset(quad_state_);
  goal_point_(0) = uniform_dist_(random_gen_) * 6.0;
  goal_point_(1) = uniform_dist_(random_gen_) + 24.0;
  goal_point_(2) = uniform_dist_(random_gen_) + 2.0;
  direction_ = goal_point_ - quad_state_.p;
  direction_.normalize();
  pre_direction_ = direction_;
  // obtain observations
  getObs(obs);
  round++;
  return true;
}

bool AvoidVisionEnv::reset(Ref<Vector<>> obs, bool random) { return reset(obs); }

bool AvoidVisionEnv::getObs(Ref<Vector<>> obs)
{
  if (obs.size() != obs_dim_ - avoidenv::kNLatent) {
    logger_.error("Observation dimension mismatch. %d != %d", obs.size(),
                  obs_dim_ - avoidenv::kNLatent);
    return false;
  }
  obs.segment<avoidenv::kNObs>(avoidenv::kObs) <<  direction_, quad_state_.v;
  return true;
}

bool AvoidVisionEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                        Ref<Vector<>> reward) {
  if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows())
    return false;
  pre_quad_state_ = quad_state_;
  pre_direction_ = direction_;
  collision_happened = false;
  double input_sum = 0;
  double occupied_distance = 0;
  for (int i=0; i<avoidenv::kNSeq; i++)
  {
    pi_act_ = act.segment<avoidenv::kNAct>(avoidenv::kAct + i*avoidenv::kNAct).
      cwiseProduct(act_std_) + act_mean_;
    pi_act_seq_.segment<avoidenv::kNAct>(avoidenv::kAct + i*avoidenv::kNAct) = pi_act_;
    if(action_mode_ == 0)
    {
      quad_state_.v = pi_act_.segment<3>(avoidenv::kAct);
      quad_state_.p = quad_state_.p + quad_state_.v * sim_dt_; 
      if(i==0)
        input_sum = (pi_act_.segment<3>(avoidenv::kAct) - pre_quad_state_.v).norm();
      else
        input_sum += (pi_act_seq_.segment<3>(avoidenv::kAct + i*avoidenv::kNAct) - 
          pi_act_seq_.segment<3>(avoidenv::kAct + (i-1)*avoidenv::kNAct)).norm();
    }
    else
    {
      quad_state_.a = pi_act_.segment<3>(avoidenv::kAct);
      quad_state_.p = quad_state_.p + quad_state_.v * sim_dt_ + 0.5 * quad_state_.a * sim_dt_ * sim_dt_;
      quad_state_.v = quad_state_.v + quad_state_.a * sim_dt_;
      if(i==0)
        input_sum = (pi_act_.segment<3>(avoidenv::kAct) - pre_quad_state_.a).norm();
      else
        input_sum += (pi_act_seq_.segment<3>(avoidenv::kAct + i*avoidenv::kNAct) - 
          pi_act_seq_.segment<3>(avoidenv::kAct + (i-1)*avoidenv::kNAct)).norm();
    }
    // std::cout<<"quad_state_.a: "<<quad_state_.a.transpose()<<"    "<<quad_state_.p.transpose()<<std::endl;
    quad_state_.t += sim_dt_;
    // std::cout<<"quad_state_.t: "<<quad_state_.t<<std::endl;
    // check the collision state of each sequence
    if(!collision_happened)
      collision_happened = LineCollisionCheck(pre_quad_state_.p, quad_state_.p);
    // occupied_distance = occupied_distance + env_ptr_->getOccupiedDistance(quad_state_.p, 2.0, 15);
    Vector<3> euler;
    quaternionToEuler(quad_state_.q(), euler);
    euler.z() = euler.z() + pi_act_(avoidenv::kNAct - 1)*i;
    if(euler.z() > M_PI) euler.z() = euler.z() - 2*M_PI;
    else if(euler.z() < -M_PI) euler.z() = euler.z() + 2*M_PI;
    Quaternion quat;
    EularToquaternion(quat, euler);
    quad_state_.q(quat);
  }
  occupied_distance = occupied_distance / avoidenv::kNSeq;
  // std::cout<<"occupied_distance: "<<occupied_distance<<std::endl;
  // if(collision_happened)
  //   std::cout<<"collision happened"<<std::endl;
  quad_ptr_->setState(quad_state_);
  if((goal_point_ - quad_state_.p).norm()>2.0)
    direction_ = (goal_point_ - quad_state_.p).normalized();
  else
    direction_ = (goal_point_ - quad_state_.p) / 2.0;
  getObs(obs);

  // reward for processing
  double process;
  Vector<3> step = (quad_state_.p - pre_quad_state_.p);
  process = step.dot(pre_direction_);
  // std::cout<<"process: "<<process<<std::endl;
  // std::cout<<"step: "<<step.transpose()<<" direct: "<<pre_direction_.transpose()<<std::endl;
  double process_reward = goal_coeff_ * process;
  if((goal_point_ - quad_state_.p).norm()<2.0)
    process_reward = goal_coeff_ * ((goal_point_ - pre_quad_state_.p).norm() - (goal_point_ - quad_state_.p).norm());
  // penalty for collision
  // const double collision_penalty = colli_coeff_ / (1.0 + occupied_distance);
  const double collision_penalty = 2.0 * colli_coeff_ * collision_happened;
  // penalty for environment risk
  const double risk_penalty = 0;

  // penalty for changes of input
  Scalar input_limit_factor;
  if(action_mode_ == 0)
    input_limit_factor = 0.4;
  else
    input_limit_factor = 0.5;
  
  double input_penalty = 0;
  if(input_sum>avoidenv::kNSeq * input_limit_factor)
    input_penalty = 20.0 * input_coeff_ * input_sum;
  else
    input_penalty = input_coeff_ * input_sum;

    // - velocity tracking (negative)
  // std::cout<<"quad_state_.v.norm(): "<<quad_state_.v.norm()<<std::endl;
  double lin_vel_penalty = 0;
  if(quad_state_.v.norm() > 6.0)
    lin_vel_penalty = lin_vel_coeff_ * (std::abs(quad_state_.v.norm() - 6.0));

  // total reward
  double total_reward = 0;
  if(action_mode_ == 0)
  {
    total_reward = process_reward + collision_penalty + risk_penalty + input_penalty;
    reward << process_reward, collision_penalty, risk_penalty, input_penalty, total_reward;  
  } else
  {
    total_reward = process_reward + collision_penalty + risk_penalty + input_penalty + lin_vel_penalty;
    reward << process_reward, collision_penalty, risk_penalty, input_penalty, lin_vel_penalty, total_reward; 
  }

  return true;
}

bool AvoidVisionEnv::LineCollisionCheck(Vector<3> pt1, Vector<3> pt2)
{
  int check_steps = std::ceil((pt2-pt1).norm() / env_ptr_->bounding.resolution_);

  for(int i=0; i<check_steps; i++)
  {
    Eigen::Vector3d checked_pt = pt1 + (pt2 - pt1).normalized() * i * env_ptr_->bounding.resolution_;
    if (env_ptr_->checkOccupied(checked_pt)) return true;
  }
  return false;
}

bool AvoidVisionEnv::setPointClouds(const std::shared_ptr<Environment> env_ptr)
{
  env_ptr_ = env_ptr;
  return true;
}

bool AvoidVisionEnv::isTerminalState(double &reward) {
  if (quad_state_.x(QS::POSX) < world_box_(0,0) ||
      quad_state_.x(QS::POSX) > world_box_(0,1) ||
      quad_state_.x(QS::POSY) < world_box_(1,0) ||
      quad_state_.x(QS::POSY) > world_box_(1,1) ||
      quad_state_.x(QS::POSZ) < world_box_(2,0) ||
      quad_state_.x(QS::POSZ) > world_box_(2,1)) {
    reward = -1.0;
    return true;
  }

  if (quad_state_.t > 120.0)
  {
    reward = -1.0;
    return true;
  }

  if (std::abs(quad_state_.x(QS::POSX)-goal_point_.x()) < 0.2 &&
      std::abs(quad_state_.x(QS::POSY)-goal_point_.y()) < 0.2 &&
      std::abs(quad_state_.x(QS::POSZ)-goal_point_.z()) < 0.2)
  {
    std::cout<<"reach goal"<<std::endl;
    reward = 1.0;
    return true;
  }

  return false;
}

bool AvoidVisionEnv::getQuadAct(Ref<Vector<>> act) const {
  if (pi_act_seq_.allFinite() && (act.size() == pi_act_seq_.size())) {
    act = pi_act_seq_;
    return true;
  }
  return false;
}

bool AvoidVisionEnv::getQuadState(Ref<Vector<>> obs) const {
  if (quad_state_.t >= 0.0 && (obs.rows() == avoidenv::kNState)) {
    obs << quad_state_.t, quad_state_.p, quad_state_.qx, quad_state_.v, goal_point_;
    return true;
  }
  logger_.error("Get Quadrotor state failed.");
  return false;
}

bool AvoidVisionEnv::getCollisionState() const {
  return quad_ptr_->getCollision();
}

bool AvoidVisionEnv::getDepthImage(Ref<DepthImgVector<>> depth_img) {
  if (!rgb_camera_ || !rgb_camera_->getEnabledLayers()[0]) {
    logger_.error(
      "No RGB Camera or depth map is not enabled. Cannot retrieve depth "
      "images.");
    return false;
  }
  bool has_img = rgb_camera_->getDepthMap(depth_img_);

  depth_img = Map<DepthImgVector<>>((float_t *)depth_img_.data,
                                    depth_img_.rows * depth_img_.cols);
  return has_img;
}

bool AvoidVisionEnv::getImage(Ref<ImgVector<>> img, const bool rgb) {
  if (!rgb_camera_) {
    logger_.error("No Camera! Cannot retrieve Images.");
    return false;
  }

  bool has_img = rgb_camera_->getRGBImage(rgb_img_);

  if (rgb_img_.rows != img_height_ || rgb_img_.cols != img_width_) {
    logger_.error(
      "Image resolution mismatch. Aborting.. Image rows %d != %d, Image cols "
      "%d != %d",
      rgb_img_.rows, img_height_, rgb_img_.cols, img_width_);
    return false;
  }

  img = Map<ImgVector<>>(rgb_img_.data, rgb_img_.rows * rgb_img_.cols *
                                            rgb_camera_->getChannels());
  return has_img;
}

bool AvoidVisionEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["simulation"]) {
    sim_dt_ = cfg["simulation"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["simulation"]["max_t"].as<Scalar>();
    action_mode_ = cfg["simulation"]["action_mode"].as<int>();
    std::vector<Scalar> act_max = cfg["simulation"]["act_max"].as<std::vector<Scalar>>();
    std::vector<Scalar> act_min = cfg["simulation"]["act_min"].as<std::vector<Scalar>>();
    act_mean_ << (act_max[0]+act_min[0])/2, (act_max[1]+act_min[1])/2, 
                  (act_max[2]+act_min[2])/2, (act_max[3]+act_min[3])/2;
    act_std_  << (act_max[0]-act_min[0])/2, (act_max[1]-act_min[1])/2, 
                  (act_max[2]-act_min[2])/2, (act_max[3]-act_min[3])/2;

  } else {
    logger_.error("Cannot load [quadrotor_env] parameters");
    return false;
  }
  if (cfg["rewards"]) {
    // load reinforcement learning related parameters
    colli_coeff_ = cfg["rewards"]["colli_coeff"].as<Scalar>();
    risk_coeff_ = cfg["rewards"]["risk_coeff"].as<Scalar>();
    goal_coeff_ = cfg["rewards"]["goal_coeff"].as<Scalar>();
    input_coeff_ = cfg["rewards"]["input_coeff"].as<Scalar>();
    lin_vel_coeff_ = cfg["rewards"]["lin_vel_coeff"].as<Scalar>();
    // load reward settings
    reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();

    rew_dim_ = cfg["rewards"]["names"].as<std::vector<std::string>>().size();
  } else {
    logger_.error("Cannot load [rewards] parameters");
    return false;
  }
  if (cfg["unity"]) {
    is_training = cfg["unity"]["render"].as<bool>();
  } else {
    logger_.error("Cannot load [unity] parameters");
    return false;
  }

  return true;
}

bool AvoidVisionEnv::configCamera(const YAML::Node &cfg,
                                const std::shared_ptr<RGBCamera> camera) {
  if (!cfg["rgb_camera"]) {
    logger_.error("Cannot config RGB Camera");
    return false;
  }

  if (!cfg["rgb_camera"]["on"].as<bool>()) {
    logger_.warn("Camera is off. Please turn it on.");
    return false;
  }

  if (quad_ptr_->getNumCamera() >= 1) {
    logger_.warn("Camera has been added. Skipping the camera configuration.");
    return false;
  }

  // create camera
  rgb_camera_ = std::make_shared<RGBCamera>();

  // load camera settings
  std::vector<Scalar> t_BC_vec =
    cfg["rgb_camera"]["t_BC"].as<std::vector<Scalar>>();
  std::vector<Scalar> r_BC_vec =
    cfg["rgb_camera"]["r_BC"].as<std::vector<Scalar>>();

  //
  Vector<3> t_BC(t_BC_vec.data());
  Matrix<3, 3> r_BC =
    (AngleAxis(r_BC_vec[2] * M_PI / 180.0, Vector<3>::UnitZ()) *
     AngleAxis(r_BC_vec[1] * M_PI / 180.0, Vector<3>::UnitY()) *
     AngleAxis(r_BC_vec[0] * M_PI / 180.0, Vector<3>::UnitX()))
      .toRotationMatrix();
  std::vector<bool> post_processing = {false, false, false};
  post_processing[0] = cfg["rgb_camera"]["enable_depth"].as<bool>();
  post_processing[1] = cfg["rgb_camera"]["enable_segmentation"].as<bool>();
  post_processing[2] = cfg["rgb_camera"]["enable_opticalflow"].as<bool>();

  //
  rgb_camera_->setFOV(cfg["rgb_camera"]["fov"].as<Scalar>());
  rgb_camera_->setWidth(cfg["rgb_camera"]["width"].as<int>());
  rgb_camera_->setChannels(cfg["rgb_camera"]["channels"].as<int>());
  rgb_camera_->setHeight(cfg["rgb_camera"]["height"].as<int>());
  rgb_camera_->setRelPose(t_BC, r_BC);
  rgb_camera_->setPostProcessing(post_processing);

  return true;
}

bool AvoidVisionEnv::addQuadrotorToUnity(
  const std::shared_ptr<UnityBridge> bridge) {
  if (!quad_ptr_) return false;
  bridge->addQuadrotor(quad_ptr_);
  return true;
}

std::ostream &operator<<(std::ostream &os, const AvoidVisionEnv &quad_env) {
  os.precision(3);
  os << "Quadrotor Environment:\n"
     << "obs dim =            [" << quad_env.obs_dim_ << "]\n"
     << "act dim =            [" << quad_env.act_dim_ << "]\n"
     << "sim dt =             [" << quad_env.sim_dt_ << "]\n"
     << "max_t =              [" << quad_env.max_t_ << "]\n"
     << "act_mean =           [" << quad_env.act_mean_.transpose() << "]\n"
     << "act_std =            [" << quad_env.act_std_.transpose() << "]\n"
     << "obs_mean =           [" << quad_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << quad_env.obs_std_.transpose() << "]"
     << std::endl;
  os.precision();
  return os;
}

}