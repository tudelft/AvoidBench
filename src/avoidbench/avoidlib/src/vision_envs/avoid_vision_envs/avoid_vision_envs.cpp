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
  map_traversability_ = 0.0;

  world_box_ << -10, 10, -3.0, 36, 0.4, 4.5;

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
      if(!use_stereo_vision_)
      {
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
      else
      {
        cfg_["rgb_camera"]["t_BC"][1] = -0.10;
        cfg_["rpg_camera"]["enable_depth"] = "no";
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

        cfg_["rgb_camera"]["t_BC"][1] = 0.10;
        right_rgb_camera_ = std::make_shared<RGBCamera>();
        if (!configCamera(cfg_, right_rgb_camera_)) {
          logger_.error(
            "Cannot config right RGB Camera. Something wrong with the config file");
        };
        quad_ptr_->addRGBCamera(right_rgb_camera_);
        //
        right_rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                                  CV_MAKETYPE(CV_8U, right_rgb_camera_->getChannels()));
        sgm_.reset(new sgm_gpu::SgmGpu(img_width_, img_height_));
      }
    }
  }
}

AvoidVisionEnv::~AvoidVisionEnv() {}

void AvoidVisionEnv::setQuadFromPtr(const std::shared_ptr<UnityBridge> bridge)
{
  quad_ptr_ = bridge->getQuadrotor(0);
  if (use_camera_)
  {
    if(!use_stereo_vision_)
    {
      rgb_camera_ = quad_ptr_->getCameras()[0];
      img_width_ = rgb_camera_->getWidth();
      img_height_ = rgb_camera_->getHeight();
      rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                                CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
      depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    }
    else
    {
      rgb_camera_ = quad_ptr_->getCameras()[0];
      right_rgb_camera_ = quad_ptr_->getCameras()[1];
      img_width_ = rgb_camera_->getWidth();
      img_height_ = rgb_camera_->getHeight();
      rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                                CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
      right_rgb_img_ = cv::Mat::zeros(img_height_, img_width_,
                                CV_MAKETYPE(CV_8U, right_rgb_camera_->getChannels()));
      sgm_.reset(new sgm_gpu::SgmGpu(img_width_, img_height_));
    }

  }
}

bool AvoidVisionEnv::reset(Ref<Vector<>> obs) {
  // reset position
  quad_state_.setZero();
  Vector<3> euler;
  euler.x() = 0.0;
  euler.y() = 0.0;
  euler.z() = 0.0;
  if (reverse && !(env_ptr_->checkOccupied(Eigen::Vector3d(last_start_point_(0), last_start_point_(1), last_start_point_(2)+0.5), 1.5))
              && !(env_ptr_->checkOccupied(Eigen::Vector3d(last_goal_point_(0), last_goal_point_(1), last_goal_point_(2)+0.5), 1.5)))
  {
    quad_state_.p.x() = last_goal_point_(0);
    quad_state_.p.y() = last_goal_point_(1);
    quad_state_.p.z() = last_goal_point_(2);
    if (last_yaw_ <= 0.0)
      euler.z() = last_yaw_ + M_PI;
    else
      euler.z() = last_yaw_ - M_PI;
    goal_point_(0) = last_start_point_(0);
    goal_point_(1) = last_start_point_(1);
    goal_point_(2) = last_start_point_(2);
    reverse = false;
  }
  else
  {
    do
    {
      int area_id = std::rand() % 3;
      if(scene_id_ == 3)
      {
        if(area_id == 0)
        {
          quad_state_.p.x() = start_area_[1] - start_area_[0]/2.0 + 
            (start_area_[0] - 2*start_area_[1]) * std::abs(uniform_dist_(random_gen_));
          quad_state_.p.y() = start_area_[1] * std::abs(uniform_dist_(random_gen_));
          quad_state_.p.z() = 3.0;
          euler.z() = 0.0;

          goal_point_(0) = end_area_[1] - end_area_[0]/2.0 + 
            (end_area_[0] - 2 * end_area_[1]) * std::abs(uniform_dist_(random_gen_));
          goal_point_(1) = - end_area_[1] * std::abs(uniform_dist_(random_gen_)) + end_origin_[1];
          goal_point_(2) = 3.0 + std::abs(uniform_dist_(random_gen_));
        } else if(area_id == 1)
        {
          quad_state_.p.x() = start_area_[1] * std::abs(uniform_dist_(random_gen_)) - 
                              start_area_[0] / 2.0;
          quad_state_.p.y() = start_area_[1] + (start_area_[1] + 
                              (start_area_[0]-2 * start_area_[1])) * std::abs(uniform_dist_(random_gen_));
          quad_state_.p.z() = 3.0;
          euler.z() = -M_PI / 2.0;

          goal_point_(0) = - end_area_[1] * std::abs(uniform_dist_(random_gen_)) + end_area_[0] / 2.0;
          goal_point_(1) = end_area_[1] + (end_area_[0] - 2 * end_area_[1]) * std::abs(uniform_dist_(random_gen_));
          goal_point_(2) = 3.0 + std::abs(uniform_dist_(random_gen_));
        } else if(area_id == 2)
        // {
        //   quad_state_.p.x() = end_area_[1] - end_area_[0]/2.0 + 
        //     (end_area_[0] - 2 * end_area_[1]) * std::abs(uniform_dist_(random_gen_));
        //   quad_state_.p.y() = - end_area_[1] * std::abs(uniform_dist_(random_gen_)) + end_area_[0];
        //   quad_state_.p.z() = 3.0;
        //   euler.z() = M_PI;

        //   goal_point_(0) = start_area_[1] - start_area_[0] / 2.0 + 
        //     (start_area_[0] - 2 * start_area_[1]) * std::abs(uniform_dist_(random_gen_));
        //   goal_point_(1) = start_area_[1] * std::abs(uniform_dist_(random_gen_));
        //   goal_point_(2) = 3.0 + std::abs(uniform_dist_(random_gen_));
        // } else if(area_id == 3)
        {
          quad_state_.p.x() = - end_area_[1] * std::abs(uniform_dist_(random_gen_)) + end_area_[0]/2.0;
          quad_state_.p.y() = (end_area_[1] + (end_area_[0] - 2.0 * end_area_[1])) * std::abs(uniform_dist_(random_gen_));
          quad_state_.p.z() = 3.0;
          euler.z() = M_PI / 2.0;

          goal_point_(0) = start_area_[1] * std::abs(uniform_dist_(random_gen_)) - start_area_[0]/2.0;
          goal_point_(1) = (start_area_[1] + (start_area_[0] - 2.0 * start_area_[1])) * std::abs(uniform_dist_(random_gen_));
          goal_point_(2) = 3.0 + std::abs(uniform_dist_(random_gen_));
        }
      }
      else if (scene_id_ == 1)
      {
        quad_state_.p.x() = - start_area_[0] / 2.0 + start_area_[0] * std::abs(uniform_dist_(random_gen_));
        quad_state_.p.y() = start_area_[1] * std::abs(uniform_dist_(random_gen_));
        quad_state_.p.z() = 3.0;

        goal_point_(0) = - end_area_[0] / 2.0 + end_area_[0] * std::abs(uniform_dist_(random_gen_));
        goal_point_(1) = - end_area_[1] * std::abs(uniform_dist_(random_gen_)) + end_origin_[1];
        goal_point_(2) = 3.0 + std::abs(uniform_dist_(random_gen_));
      }
    } while (env_ptr_->checkOccupied(Eigen::Vector3d(quad_state_.p.x(), quad_state_.p.y(), quad_state_.p.z()+0.5), 1.5) ||
    env_ptr_->checkOccupied(Eigen::Vector3d(goal_point_(0), goal_point_(1), goal_point_(2)+0.5), 1.5));

    last_goal_point_ = goal_point_;
    last_start_point_ << quad_state_.p.x(), quad_state_.p.y(), quad_state_.p.z();
    last_yaw_ = euler.z();
    // reverse = true;
  }

  // reset velocity
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
  Quaternion quat;
  EularToquaternion(quat, euler);
  quad_state_.x(QS::ATTW) = quat.w();
  quad_state_.x(QS::ATTX) = quat.x();
  quad_state_.x(QS::ATTY) = quat.y();
  quad_state_.x(QS::ATTZ) = quat.z();

  pre_quad_state_ = quad_state_;
  // reset quadrotor with random states
  quad_ptr_->reset(quad_state_);

  log_distance_ = log(sqrt(pow(goal_point_.x()-quad_state_.p.x(), 2) + pow(goal_point_.y()-quad_state_.p.y(), 2)) + 1);
  pre_log_distance_ = log_distance_;
  horizon_vel_ = 0.0;
  pre_horizon_vel_ = 0.0;

  theta_ = atan2(-(goal_point_ - quad_state_.p).x(), (goal_point_ - quad_state_.p).y());
  pre_theta_ = theta_;
  horizon_vel_dire_ = 0.0;
  pre_horizon_vel_dire_ = 0.0;
  yaw = euler.z();
  // obtain observations
  getObs(obs);

  round++;
  return true;
}

bool AvoidVisionEnv::reset(Ref<Vector<>> obs, bool random) { 
  if (!random)
    random_gen_.seed(seed_);
  return reset(obs); 
  }

bool AvoidVisionEnv::getObs(Ref<Vector<>> obs)
{
  if (obs.size() != obs_dim_ - avoidenv::kNLatent) {
    logger_.error("Observation dimension mismatch. %d != %d", obs.size(),
                  obs_dim_ - avoidenv::kNLatent);
    return false;
  }
  obs.segment<avoidenv::kNObs>(avoidenv::kObs) << log_distance_, horizon_vel_, theta_, horizon_vel_dire_,
                                                   goal_point_.z() - quad_state_.p.z(), quad_state_.v.z(), yaw;
  return true;
}

bool AvoidVisionEnv::body2world(Ref<Vector<>> body, Ref<Vector<>> world)
{
  if (body.size() != 3 || world.size() != 3)
  {
    logger_.error("body2world dimension mismatch. %d != %d", body.size(), world.size());
    return false;
  }
  Quaternion quat;
  quat.w() = quad_state_.x(QS::ATTW);
  quat.x() = quad_state_.x(QS::ATTX);
  quat.y() = quad_state_.x(QS::ATTY);
  quat.z() = quad_state_.x(QS::ATTZ);
  Matrix<3, 3> rot_mat = quat.toRotationMatrix();
  Vector<3> world_flu = rot_mat * body;
  world << -world_flu.y(), world_flu.x(), world_flu.z(); // FLU to RFU
  return true;
}

bool AvoidVisionEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs,
                        Ref<Vector<>> reward) {
  if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows())
    return false;

  pre_quad_state_ = quad_state_;
  pre_log_distance_ = log_distance_;
  pre_horizon_vel_ = horizon_vel_;
  pre_theta_ = theta_;
  pre_horizon_vel_dire_ = horizon_vel_dire_;
  collision_happened = false;
  // double input_sum = 0;
  double occupied_distance = 0;
  for (int i=0; i<avoidenv::kNSeq; i++)
  {
    pi_act_ = act.segment<avoidenv::kNAct>(avoidenv::kAct + i*avoidenv::kNAct).
      cwiseProduct(act_std_) + act_mean_;
    // std::cout<<"pi_act_: "<<pi_act_.transpose()<<std::endl;
    pi_act_seq_.segment<avoidenv::kNAct>(avoidenv::kAct + i*avoidenv::kNAct) = pi_act_;
    Vector<3> action_pos, action_pos_world;
    action_pos = pi_act_.segment<3>(avoidenv::kAct);
    body2world(action_pos, action_pos_world);

    if(action_mode_ == 0)
    {
      quad_state_.v = action_pos; //update velocity of body frame
      quad_state_.p = quad_state_.p + action_pos_world * sim_dt_; //update position of world frame
      // if(i==0)
      //   input_sum = (pi_act_.segment<3>(avoidenv::kAct) - pre_quad_state_.v).norm();
      // else
      //   input_sum += (pi_act_seq_.segment<3>(avoidenv::kAct + i*avoidenv::kNAct) - 
      //     pi_act_seq_.segment<3>(avoidenv::kAct + (i-1)*avoidenv::kNAct)).norm();
    }
    else
    {
      quad_state_.a = action_pos; //update acceleration of body frame
      Vector<3> v_world;
      body2world(quad_state_.v, v_world);
      quad_state_.p = quad_state_.p + v_world * sim_dt_ + 0.5 * action_pos_world * sim_dt_ * sim_dt_; //update position of world frame
      quad_state_.v = quad_state_.v + quad_state_.a * sim_dt_; //update velocity of body frame
      // if(i==0)
      //   input_sum = (pi_act_.segment<3>(avoidenv::kAct) - pre_quad_state_.a).norm();
      // else
      //   input_sum += (pi_act_seq_.segment<3>(avoidenv::kAct + i*avoidenv::kNAct) - 
      //     pi_act_seq_.segment<3>(avoidenv::kAct + (i-1)*avoidenv::kNAct)).norm();
    }
    quad_state_.t += sim_dt_;
    // check the collision state of each sequence
    if(!collision_happened)
      collision_happened = LineCollisionCheck(pre_quad_state_.p, quad_state_.p);
    // occupied_distance = occupied_distance + env_ptr_->getOccupiedDistance(quad_state_.p, 2.0, 15);
    Vector<3> euler;
    quaternionToEuler(quad_state_.q(), euler);

    euler.z() = euler.z() + pi_act_(avoidenv::kNAct - 1) * sim_dt_;
    if(euler.z() > M_PI) euler.z() = euler.z() - 2*M_PI;
    else if(euler.z() < -M_PI) euler.z() = euler.z() + 2*M_PI;
    yaw = euler.z();

    /********** if we don't consider the attitude of pitch and roll ***********/ 
    // Quaternion quat;
    // EularToquaternion(quat, euler);
    // quad_state_.q(quat);

    // if we enable the attitude of pitch and roll
    Eigen::Vector3d thrust =  Eigen::Vector3d(-quad_state_.a[1], quad_state_.a[0], quad_state_.a[2]) + 9.81 * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
    Eigen::Quaterniond q_pitch_roll = Eigen::Quaterniond::FromTwoVectors(I_eZ_I, thrust);

    Eigen::Quaterniond q_heading = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_att = q_pitch_roll * q_heading;
    q_att.normalize();
    quad_state_.q(q_att);

  }
  // occupied_distance = occupied_distance / avoidenv::kNSeq;

  quad_ptr_->setState(quad_state_);
  double x = (goal_point_ - quad_state_.p).x();
  double y = (goal_point_ - quad_state_.p).y();
  double z = (goal_point_ - quad_state_.p).z();
  // direction_ = (goal_point_ - quad_state_.p).normalized();
  // std::cout<<"quad_state_.p: "<<quad_state_.p.transpose()<<std::endl;
  log_distance_ = log(sqrt(x*x + y*y) + 1.0);
  // d_distance_ = (-2*x*quad_state_.v.x() - 2*y*quad_state_.v.y()
  //                - 2*z*quad_state_.v.z()) / (goal_point_ - quad_state_.p).norm();
  horizon_vel_ = sqrt(pow(quad_state_.v.x(), 2) + pow(quad_state_.v.y(), 2));

  theta_ = atan2(-x, y); // world frame is RFU
  // d_theta_ = (quad_state_.v.x() * y - quad_state_.v.y() * x) / (x*x + y*y);
  horizon_vel_dire_ = atan2(quad_state_.v.y(), quad_state_.v.x());
  getObs(obs);

  // reward for goal
  const double goal_reward = distance_coeff_ * log_distance_;
  //penalty for speed
  double speed_penalty = 0;
  if (std::abs(horizon_vel_) > 3.0)
    speed_penalty = vel_coeff_ * std::abs(horizon_vel_);
  // speed_penalty = vel_coeff_ * std::abs(horizon_vel_ - 2.0);

  //penalty for vertical
  const double vertical_penalty = vert_coeff_ * (std::abs(quad_state_.p.z() - goal_point_.z()));
  //penalty for angular velocity
  const double angular_penalty = angle_vel_coeff_ * std::abs(horizon_vel_dire_ + yaw - theta_);

  //penalty for collision
  // const double collision_penalty = colli_coeff_ * collision_happened;

  //penalty for input
  const double input_penalty = input_coeff_ * (quad_state_.a - pre_quad_state_.a).norm();

  //penalty for yaw error
  const double yaw_penalty = yaw_coeff_ * std::abs(horizon_vel_dire_);

  double total_reward = 0;

  total_reward = goal_reward + speed_penalty + vertical_penalty + angular_penalty + input_penalty + yaw_penalty;
  reward << goal_reward, speed_penalty, vertical_penalty, angular_penalty, input_penalty, yaw_penalty,total_reward; 

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
    reward = -2.0;
    return true;
  }

  if (quad_state_.t > 100.0)
  {
    if (scene_id_ == 1)
      reward = -10.0;
    else if (scene_id_ == 3)
      reward = -5.0;
    return true;
  }

  if (collision_happened)
  {
    if (scene_id_ == 1)
      reward = -10.0;
    else if (scene_id_ == 3)
      reward = -2.0;
    if (reset_if_collide_)
    {
      return true;
    }
  }

  if (std::abs(quad_state_.x(QS::POSX)-goal_point_.x()) < 0.4 &&
      std::abs(quad_state_.x(QS::POSY)-goal_point_.y()) < 0.4 &&
      std::abs(quad_state_.x(QS::POSZ)-goal_point_.z()) < 0.4)
  {
    std::cout<<"reach goal"<<std::endl;
    reward = 20.0 / map_traversability_;
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
  if (!rgb_camera_) {
    logger_.error(
      "No RGB Camera or depth map is not enabled. Cannot retrieve depth "
      "images.");
    return false;
  }

  bool has_img;
  if (!use_stereo_vision_)
  {
    has_img = rgb_camera_->getDepthMap(depth_img_);
    depth_img = Map<DepthImgVector<>>((float_t *)depth_img_.data, depth_img_.rows * depth_img_.cols);
  }
  else 
  {
    if (!right_rgb_camera_) {
    logger_.error(
      "No Right RGB Camera. Cannot retrieve depth images.");
    return false;
    }

    has_img = right_rgb_camera_->getRGBImage(right_rgb_img_) && rgb_camera_->getRGBImage(rgb_img_);

    // show left and right images at the same time
    // cv::Mat stereo_img(img_height_, img_width_ * 2, CV_8UC3);
    // cv::Mat left(stereo_img, cv::Rect(0, 0, img_width_, img_height_));
    // cv::Mat right(stereo_img, cv::Rect(img_width_, 0, img_width_, img_height_));
    // rgb_img_.copyTo(left);
    // right_rgb_img_.copyTo(right);
    // //save image
    // std::string img_name = "/home/hyyu/RL_avoiding_private/src/learning/" + std::to_string(round) + ".png";
    // cv::imwrite(img_name, stereo_img);
    
    depth_img = computeDepthImage(rgb_img_, right_rgb_img_);
  }

  return has_img;
}

Ref<DepthImgVector<>> AvoidVisionEnv::computeDepthImage(const cv::Mat& left_frame, const cv::Mat& right_frame)
{
  cv::Mat disp = cv::Mat(img_height_, img_width_, CV_8UC1);
  sgm_->computeDisparity(left_frame, right_frame, &disp);
  disp.convertTo(disp, CV_32FC1);
  // compute depth from disparity
  cv::Mat depth_float(img_height_, img_width_, CV_32FC1);
  float f = (img_width_ / 2.0) / std::tan(M_PI * (rgb_camera_->getFOV() / 2.0) / 180.0);
  for (int r = 0; r<img_height_; ++r) {
    for (int c = 0; c<img_width_; ++c) {
      if(disp.at<float>(r, c) == 0.0f)
        depth_float.at<float>(r, c) = 0.0f;
      else if (disp.at<float>(r, c) == 255.0f)
        depth_float.at<float>(r, c) = 255.0f;
      else
        depth_float.at<float>(r, c) = 0.2f * f / disp.at<float>(r, c);
    }
  }
  return Map<DepthImgVector<>>((float_t *)depth_float.data, depth_float.rows * depth_float.cols);
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
    reset_if_collide_ = cfg["simulation"]["reset_if_collide"].as<bool>();
    use_stereo_vision_ = cfg["simulation"]["use_stereo_vision"].as<bool>();

  } else {
    logger_.error("Cannot load [quadrotor_env] parameters");
    return false;
  }
  if (cfg["rewards"]) {
    // load reinforcement learning related parameters
    colli_coeff_ = cfg["rewards"]["colli_coeff"].as<Scalar>();
    distance_coeff_ = cfg["rewards"]["distance_coeff"].as<Scalar>();
    vel_coeff_ = cfg["rewards"]["vel_coeff"].as<Scalar>();
    vert_coeff_ = cfg["rewards"]["vert_coeff"].as<Scalar>();
    angle_vel_coeff_ = cfg["rewards"]["angle_vel_coeff"].as<Scalar>();
    input_coeff_ = cfg["rewards"]["input_coeff"].as<Scalar>();
    yaw_coeff_ = cfg["rewards"]["yaw_coeff"].as<Scalar>();
    // angle_coeff_ = cfg["rewards"]["angle_coeff"].as<Scalar>();
    // load reward settings
    reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();

    rew_dim_ = cfg["rewards"]["names"].as<std::vector<std::string>>().size();
  } else {
    logger_.error("Cannot load [rewards] parameters");
    return false;
  }
  if (cfg["unity"]) {
    is_training = cfg["unity"]["render"].as<bool>();
    scene_id_ = cfg["unity"]["scene_id"].as<int>();
    start_area_ = cfg["unity"]["start_area"].as<std::vector<Scalar>>();
    end_area_ = cfg["unity"]["end_area"].as<std::vector<Scalar>>();
    start_origin_ = cfg["unity"]["start_origin"].as<std::vector<Scalar>>();
    end_origin_ = cfg["unity"]["end_origin"].as<std::vector<Scalar>>();
    world_box_ << -start_area_[0]/2 - 2.0, start_area_[0]/2 + 2.0, -2.0, end_origin_[1] + 8.0, 0.4, 6.0;
  } else {
    logger_.error("Cannot load [unity] parameters");
    return false;
  }

  if (cfg["simulation"]){
    seed_ = cfg["simulation"]["seed"].as<int>();
  } else {
    logger_.error("Cannot load [simulation] parameters");
    return false;
  }

  return true;
}

bool AvoidVisionEnv::resetRewCoeff(const YAML::Node &cfg)
{
  if (cfg["rewards"]) {
    // load reinforcement learning related parameters
    colli_coeff_ = cfg["rewards"]["colli_coeff_new"].as<Scalar>();
    distance_coeff_ = cfg["rewards"]["distance_coeff_new"].as<Scalar>();
    vel_coeff_ = cfg["rewards"]["vel_coeff_new"].as<Scalar>();
    vert_coeff_ = cfg["rewards"]["vert_coeff_new"].as<Scalar>();
    angle_vel_coeff_ = cfg["rewards"]["angle_vel_coeff_new"].as<Scalar>();
    input_coeff_ = cfg["rewards"]["input_coeff_new"].as<Scalar>();
    yaw_coeff_ = cfg["rewards"]["yaw_coeff_new"].as<Scalar>();
  } else {
    logger_.error("Cannot load [rewards] parameters");
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
  camera->setFOV(cfg["rgb_camera"]["fov"].as<Scalar>());
  camera->setWidth(cfg["rgb_camera"]["width"].as<int>());
  camera->setChannels(cfg["rgb_camera"]["channels"].as<int>());
  camera->setHeight(cfg["rgb_camera"]["height"].as<int>());
  camera->setRelPose(t_BC, r_BC);
  camera->setPostProcessing(post_processing);

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