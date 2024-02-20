#include "avoid_manage.hpp"

namespace avoid_manage {

AvoidManage::AvoidManage(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    unity_ready_(false),
    start_ok(false),
    start_time(0.0),
    mission_id(0),
    trial_id(0),
    mission_state(Mission_state::PREPARING) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  if(scene_id_ == 3)
    cfg_ = ros::package::getPath("avoid_manage") + "/params/task_outdoor.yaml";
  else if(scene_id_ == 1)
    cfg_ = ros::package::getPath("avoid_manage") + "/params/task_indoor.yaml";

  image_transport::ImageTransport it(nh);
  left_pub = it.advertise("/rgb/left", 1);
  if(perform_sgm_)
    right_pub = it.advertise("/rgb/right", 1);
  depth_pub = it.advertise("/depth", 1);
  depth_cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("camera_depth/camera/camera_info", 10);
  pose_pub_ = nh_.advertise<rpg_quadrotor_msgs::TrajectoryPoint>("autopilot/reset_reference_state", 10);
  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  force_hover_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/force_hover", 1);
  arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
  goal_pub_ = nh_.advertise<nav_msgs::Path>("goal_point", 10);
  task_state_pub_ = nh_.advertise<avoid_msgs::TaskState>("task_state", 100);
  avoid_metrics_pub_ = nh_.advertise<avoid_msgs::Metrics>("metrics", 10);
  collision_info_pub_ = nh_.advertise<std_msgs::Bool>("collision", 1);
  // initialize subscriber call backs
  gazebo_model_srv_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &AvoidManage::poseCallback, this);
  mission_start_sub_ = nh_.subscribe("mission_start", 10,
                                 &AvoidManage::MissionFlagCallback, this);
  iter_time_sub_ = nh_.subscribe("iter_time", 10, &AvoidManage::IterTimeCallback, this);

  if(save_data_mode)
  {
    save_pc_sub_ = nh_.subscribe("/pc_path", 1, &AvoidManage::SavePointcloudCallback, this);
    training_period_sub = nh_.subscribe("training_state", 1, &AvoidManage::SavePointcloudCallback, this);
  }

  render_loop_ = nh_.createTimer(ros::Rate(task_loop_freq_), &AvoidManage::RenderingCallback, this);
  metrics_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_), &AvoidManage::MetricsLoopCallback, this);
  task_loop_ = nh_.createTimer(ros::Rate(task_loop_freq_), &AvoidManage::MissionCallback, this);
  mission_state_loop_ = nh_.createTimer(ros::Rate(120), &AvoidManage::MissionStateLoopCallback, this);

  avoidbench_bridge = std::make_shared<AvoidbenchBridge>(cfg_);
  unity_init_ = true;
  metrics = std::make_shared<avoidmetrics::Metrics>(cfg_);
  mission_state = Mission_state::UNITYSETTING;
}

AvoidManage::~AvoidManage() {}

void AvoidManage::IterTimeCallback(const std_msgs::Float32::ConstPtr &msg)
{
  if(mission_state == Mission_state::MISSIONPROCESS ||
      mission_state == Mission_state::SENTMISSION ||
      mission_state == Mission_state::WAITMISSION)
  {
    iter_times.push_back(msg->data);
  }
}

void AvoidManage::SavePointcloudCallback(const avoid_msgs::PointCloudInfoConstPtr &msg)
{
  std::string curr_dir = msg->curr_dir;
  Eigen::Vector3d range_, origin_;

  range_.x() = msg->range[0];
  range_.y() = msg->range[1];
  range_.z() = msg->range[2];

  origin_.x() = msg->origin[0];
  origin_.y() = msg->origin[1];
  origin_.z() = msg->origin[2];
  avoidbench_bridge->getPointCloud(curr_dir, range_, origin_);
}

void AvoidManage::TrainingStateCallback(const std_msgs::BoolConstPtr &msg)
{
  if_traning_period = msg->data;
}

void AvoidManage::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  received_state_est_.orientation.x() = msg->pose.pose.orientation.x;
  received_state_est_.orientation.y() = msg->pose.pose.orientation.y;
  received_state_est_.orientation.z() = msg->pose.pose.orientation.z;
  received_state_est_.orientation.w() = msg->pose.pose.orientation.w;
  received_state_est_.position.x() = msg->pose.pose.position.x;
  received_state_est_.position.y() = msg->pose.pose.position.y;
  received_state_est_.position.z() = msg->pose.pose.position.z;
  received_state_est_.velocity.x() = msg->twist.twist.linear.x;
  received_state_est_.velocity.y() = msg->twist.twist.linear.y;
  received_state_est_.velocity.z() = msg->twist.twist.linear.z;
  received_state_est_.transformVelocityToWorldFrame();
  received_state_est_.timestamp = msg->header.stamp;

  debug_out.open(getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/data/debug_out.txt"), 
                    std::ios::out | std::ios::app);
  std::string out_data;
  out_data = std::to_string(received_state_est_.position.x()) + " " + 
              std::to_string(received_state_est_.position.y()) + " " + 
              std::to_string(received_state_est_.position.z()) + " " + 
              std::to_string(received_state_est_.velocity.x()) + " " + 
              std::to_string(received_state_est_.velocity.y()) + " " + 
              std::to_string(received_state_est_.velocity.z()) + " " + "\n";
  debug_out << out_data;
  debug_out.close();
}

void AvoidManage::RenderingCallback(const ros::TimerEvent &event)
{
  if(unity_init_)
  {
    unity_ready_ = avoidbench_bridge->updateUnity(received_state_est_);
    collision_state = avoidbench_bridge->getQuadCollisionState();
    std_msgs::Bool colli;
    colli.data = collision_state;
    collision_info_pub_.publish(colli);
    if(mission_state == Mission_state::MISSIONPROCESS  || mission_state == Mission_state::WAITMISSION)
    {
      if(last_collision_state==false && collision_state==true)
      {
        collision_happen = true;
      }
    }
    cv::Mat depth, disparity, left, right;
    ros::Time timestamp = ros::Time::now();
    if(perform_sgm_)
    {
      avoidbench_bridge->getImages(&left, &right, &depth, &disparity);
      sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
      left_msg->header.stamp = timestamp;
      left_pub.publish(left_msg);
      sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right).toImageMsg();
      right_msg->header.stamp = timestamp;
      right_pub.publish(right_msg);
      sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
      depth_msg->header.stamp = timestamp;
      depth_msg->header.frame_id = "camera_depth_optical_center_link";
      depth_pub.publish(depth_msg);
    }
    else
    {
      avoidbench_bridge->getImages(&left, &depth);
      sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
      left_msg->header.stamp = timestamp;
      left_pub.publish(left_msg);
      sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth).toImageMsg();
      depth_msg->header.stamp = timestamp;
      depth_msg->header.frame_id = "camera_depth_optical_center_link";
      depth_pub.publish(depth_msg);
    }

    //publish camera info
    sensor_msgs::CameraInfo cam_info;
    std::vector<double> D{0.0, 0.0, 0.0, 0.0, 0.0};
    boost::array<double, 9> K = {314.463, 0.0, 320.5, 0.0, 314.463, 240.5, 0.0, 0.0, 1.0};
    boost::array<double, 12> P = {314.463, 0.0, 320.5, -0.0, 0.0, 314.463, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.height = height;
    cam_info.width = width;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D = D;
    cam_info.K = K;
    cam_info.P = P;
    cam_info.R = R;
    cam_info.binning_x = 0;
    cam_info.binning_y = 0;
    cam_info.header.frame_id = "camera_depth_optical_center_link";
    cam_info.header.stamp = timestamp;
    depth_cam_info_pub.publish(cam_info);

    last_collision_state = collision_state;
  }
}

void AvoidManage::MissionFlagCallback(const std_msgs::Empty::ConstPtr &msg)
{
  if((mission_state != Mission_state::UNITYSETTING) && 
      mission_state != Mission_state::GAZEBOSETTING)
  mission_state = Mission_state::MISSIONPROCESS;
}

void AvoidManage::MetricsLoopCallback(const ros::TimerEvent &event) {
  if(!unity_ready_)
    return;
  start_time = ros::Time::now().toSec();
  if(!start_ok)
  {
    ros::Duration(2.0).sleep();
    std_msgs::Bool if_arm_;
    if_arm_.data = true;
    std_msgs::Empty takeoff;
    arm_pub_.publish(if_arm_);
    start_pub_.publish(takeoff);
    start_ok = true;
    MetricsThread();
  }
}

bool AvoidManage::TakeoffSuccess()
{
  if(received_state_est_.position.z()>1.15 && abs(received_state_est_.velocity.z())<0.05)
    return true;
  else
    return false;
}

void AvoidManage::MetricsThread()
{
  metrics->run();
  ROS_INFO("task end");
  avoid_metrics_pub_.publish(metrics->getMetricsMsg());
}

void AvoidManage::MissionCallback(const ros::TimerEvent &event)
{
  static std::shared_ptr<avoidmetrics::Mission> mission;
  static avoidlib::mission_parameter p_m;
  if(if_traning_period) return;
  
  if(mission_state == Mission_state::UNITYSETTING)
  {
    if(get_new_mission)
    {
      std::cout<<"get new mission"<<std::endl;
      //initialize the param for mission
      getMissionParam(&p_m, mission_id);

      //send the obstacles information to Unity
      // avoidbench_bridge->setParamFromMission(p_m);
      // avoidbench_bridge->spawnObstacles();
      // while(!avoidbench_bridge->ifSceneChanged())
      //   avoidbench_bridge->SpawnNewObs();

      //random generate start and end point until they both have a certain free space
      int ss=0;

      while(avoidbench_bridge->checkCollisionState(&(p_m.m_start_point), true) || 
        avoidbench_bridge->checkCollisionState(&(p_m.m_end_point)))
      {
        ss++;
        mission_state = Mission_state::UNITYSETTING;
        if_update_map = false;
        if(ss>16)
        {
          if_update_map = true;
          return;
        }
        getMissionParam(&p_m, mission_id);
        ros::Duration(0.2).sleep();
        ros::spinOnce();
      }

      // get the datasets of environment point clouds. (for environment metrics)
      if(!save_data_mode)
      {
        Eigen::Vector3d range_, origin_;
        CalculateRanges(&range_, &origin_);
        // avoidbench_bridge->getPointCloud("", range_, origin_);
      }
      ros::Duration(5.0).sleep();
    }
    else {
      //random generate start and end point until they both have a certain free space
      if_update_map = false;
      getMissionParam(&p_m, mission_id);
      while(avoidbench_bridge->checkCollisionState(&(p_m.m_start_point), true) || 
        avoidbench_bridge->checkCollisionState(&(p_m.m_end_point)))
      {
        mission_state = Mission_state::UNITYSETTING;
        if_update_map = false;
        getMissionParam(&p_m, mission_id);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
      // mission->reset(p_m);
      ros::Duration(5.0).sleep();
    }
    
    mission = std::make_shared<avoidmetrics::Mission>(cfg_, p_m, mission_id);    
    //reset the position of start point for gazebo
    // resetGazebo(Eigen::Vector3d(p_m.m_start_point[0], p_m.m_start_point[1], p_m.m_start_point[2]), p_m.m_start_point[3]);
    std::cout<<"start point: "<<p_m.m_start_point[0]<<" "<<p_m.m_start_point[1]<<" "<<p_m.m_start_point[2]<<" "<<p_m.m_start_point[3]<<std::endl;
    mission_start_time = ros::Time::now();
    ros::Duration(1.0).sleep();
    mission_state = Mission_state::SENTMISSION;
    return;
  }

  if(mission_state == Mission_state::SENTMISSION)
  {
    if(TakeoffSuccess())
    {
      nav_msgs::Path pub_msg;
      geometry_msgs::PoseStamped goal;
      goal.pose.position.x = mission->end_point.x();
      goal.pose.position.y = mission->end_point.y();
      goal.pose.position.z = mission->end_point.z();
      pub_msg.poses.push_back(goal);
      pub_msg.header.stamp = ros::Time::now();
      pub_msg.header.frame_id = "world";
      goal_pub_.publish(pub_msg);
      ros::Duration(0.1).sleep();
      if((ros::Time::now() - mission_start_time).toSec()>60.0)
      {
        ROS_WARN("Exceed 1 min cannot start planning, pass current Trial");
        mission_state = Mission_state::GAZEBOSETTING;
      }
      return;
    }
  }

  if(mission_state == Mission_state::WAITMISSION)
  {
    // mission->getTrajectory(received_state_est_);
    // if((mission->finished && !save_data_mode) || 
    // ((ros::Time::now() - mission_start_time).toSec()>80.0))
    // {
    //   mission_state = Mission_state::GAZEBOSETTING;
    //   return;
    // }
  }

  if(mission_state == Mission_state::MISSIONPROCESS)
  {
    // mission->getTrajectory(received_state_est_);
    // if(collision_happen)
    // {
    //   ROS_INFO("collision happened");
    //   mission->CollisionCount();
    //   collision_happen = false;
    // }
    // mission_state = Mission_state::WAITMISSION;
    // if(mission->stop_flag)
    //   mission_state = Mission_state::GAZEBOSETTING;
  }

  if(mission_state == Mission_state::GAZEBOSETTING)
  {
    ros::Duration(1.0).sleep();
    resetGazebo(Eigen::Vector3d(0, 0, 1.2), 0);
    std_msgs::Empty hover;
    force_hover_pub_.publish(hover);
    mission->cal_time = iter_times;
    iter_times.clear();
    std::vector<float>(iter_times).swap(iter_times);
    metrics->setMissions(mission);
    ros::Duration(1.0).sleep();
    trial_id++;
    if(trial_id == (mission->trials))
    {
      mission_id++;
      trial_id = 0;
      get_new_mission = true;
    } else
      get_new_mission = false;

    std::cout<<"mission_id: "<<mission_id<<"  trial_id: "<<trial_id<<std::endl;

    if(mission_id<mission_number)
      mission_state = Mission_state::UNITYSETTING;
    else
    {
      mission_state = Mission_state::PREPARING;
      metrics->setTaskFinishFlag(true);
    }
  }
}

void AvoidManage::MissionStateLoopCallback(const ros::TimerEvent &event)
{
  avoid_msgs::TaskState state;
  state.header.stamp = ros::Time::now();
  state.Mission_state = mission_state;
  task_state_pub_.publish(state);
}

void AvoidManage::resetGazebo(const Eigen::Vector3d &pos, const double yaw)
{
  gazebo_msgs::SetModelState dronestate;
  rpg_quadrotor_msgs::TrajectoryPoint cmd;
  dronestate.request.model_state.model_name = quad_name;
  cmd.pose.position.x = dronestate.request.model_state.pose.position.x = pos.x();
  cmd.pose.position.y = dronestate.request.model_state.pose.position.y = pos.y();
  cmd.pose.position.z = dronestate.request.model_state.pose.position.z = pos.z();
  cmd.heading = yaw;
  Eigen::Quaterniond quat = ypr2quat(yaw, 0, 0);
  dronestate.request.model_state.pose.orientation.w = quat.w();
  dronestate.request.model_state.pose.orientation.x = quat.x();
  dronestate.request.model_state.pose.orientation.y = quat.y();
  dronestate.request.model_state.pose.orientation.z = quat.z();
  dronestate.request.model_state.twist.linear.x = 0.0;
  dronestate.request.model_state.twist.linear.y = 0.0;
  dronestate.request.model_state.twist.linear.z = 0.0;
  dronestate.request.model_state.twist.angular.x = 0.0;
  dronestate.request.model_state.twist.angular.y = 0.0;
  dronestate.request.model_state.twist.angular.z = 0.0;
  dronestate.request.model_state.reference_frame = "world";

  gazebo_model_srv_.call(dronestate);

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  pose_pub_.publish(cmd);
}

Eigen::Quaterniond AvoidManage::ypr2quat(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
 
    Eigen::Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
 
    return q;
}

void AvoidManage::getMissionParam(avoidlib::mission_parameter* const m_param, const int &m_id)
{
    YAML::Node cfg = YAML::LoadFile(cfg_);

    static float width, length, radius_area, radius_origin;
    static std::vector<float> start_area, start_origin, end_area, end_origin;
    static int trials_;

    if(!get_seed)
    {
      start_area.resize(2);
      start_origin.resize(2);
      end_area.resize(2);
      end_origin.resize(2);
      env_range = cfg["unity"]["range"].as<std::vector<double>>();
      env_origin = cfg["unity"]["origin"].as<std::vector<double>>();
      start_area = cfg["mission"]["start_area"].as<std::vector<float>>();
      start_origin = cfg["mission"]["start_origin"].as<std::vector<float>>();
      end_area = cfg["mission"]["end_area"].as<std::vector<float>>();
      end_origin = cfg["mission"]["end_origin"].as<std::vector<float>>();
      radius_area = cfg["mission"]["radius_area"].as<float>();
      radius_origin = cfg["mission"]["radius_origin"].as<float>();
      int seed = cfg["mission"]["seed"].as<int>();
      trials_ = cfg["mission"]["trials"].as<int>();
      srand(seed);
      get_seed = true;
    }
    float rand = (std::rand()%200)/200.0f;
    if(scene_id_ == UnityScene::NATUREFOREST)
    {
      int area_id = std::rand()%4;
      if (area_id == 0)
        m_param->m_start_point = {-start_area[0]/2.0f+start_area[0]*rand,
                                  start_area[1]*rand,
                                  2.0f, 0};
        rand = (std::rand()%200)/200.0f;
        m_param->m_end_point = {-end_area[0]/2.0f+end_area[0]*rand,
                                  -end_area[1]*rand+end_origin[1],
                                  2.0f+1.0f*rand};
      if (area_id == 1)
      {
        m_param->m_start_point = {start_area[1]*rand-start_area[0]/2.0,
                                start_area[0]*rand,
                                2.0f, -M_PI/2.0};
        rand = (std::rand()%200)/200.0f;
        m_param->m_end_point = {-end_area[1]*rand+end_area[0]/2,
                                  end_area[0]*rand,
                                  2.0f+1.0f*rand};  
      }
      if (area_id == 2)
      {
        m_param->m_start_point = {-end_area[0]/2.0f+end_area[0]*rand,
                                  -end_area[1]*rand+end_area[0],
                                  2.0f, M_PI};
        rand = (std::rand()%200)/200.0f;
        m_param->m_end_point = {-start_area[0]/2.0f+start_area[0]*rand,
                                  start_area[1]*rand,
                                  2.0f+1.0f*rand};  
      }
      if (area_id == 3)
      {
        m_param->m_start_point = {-end_area[1]*rand+end_area[0]/2,
                                  end_area[0]*rand,
                                  2.0f, M_PI/2.0};
        rand = (std::rand()%200)/200.0f;
        m_param->m_end_point = {start_area[1]*rand-start_area[0]/2,
                                  start_area[0]*rand,
                                  2.0f+1.0f*rand};  
      }
    }

    else if(scene_id_ == UnityScene::WAREHOUSE)
    {
      m_param->m_start_point = {-start_area[0]/2.0f+start_area[0]*rand,
                                start_area[1]*rand,
                                2.0f, 0};
      rand = (std::rand()%200)/200.0f;
      m_param->m_end_point = {-end_area[0]/2.0f+end_area[0]*rand,
                                -end_area[1]*rand+end_origin[1],
                                2.0f+1.0f*rand};
    }
    
    if (!if_update_map)
    {
      if_update_map = true;
      return;
    }
    m_param->trials =trials_;
    m_param->m_radius = radius_origin + radius_area*rand;
    m_param->m_seed = std::rand()%200;
    rand = (std::rand()%200)/200.0f;
    m_param->m_opacity = rand;
    if(!save_data_mode)
      m_param->m_pc_file_name = cfg["mission"]["pc_file_name"].as<std::string>()+std::to_string(m_id);
    else
    {
      m_param->m_pc_file_name = cfg["mission"]["pc_file_name"].as<std::string>();
      mission_number = cfg["mission"]["rollout"].as<int>();
    }
    ROS_INFO("get new parameters!");

}

bool AvoidManage::loadParams() {
  // load parameters
  int env_id;
  quadrotor_common::getParam("flight_number", mission_number, pnh_);
  quadrotor_common::getParam("unity/env_idx", env_id, pnh_);
  quadrotor_common::getParam("unity/save_data_mode", save_data_mode, pnh_);
  quadrotor_common::getParam("camera/fov", fov, pnh_);
  quadrotor_common::getParam("camera/width", width, pnh_);
  quadrotor_common::getParam("camera/height", height, pnh_);
  quadrotor_common::getParam("camera/baseline", baseline, pnh_);
  quadrotor_common::getParam("camera/perform_sgm", perform_sgm_, pnh_);
  pnh_.getParam("quad_name", quad_name);
  scene_id_ = env_id;
  return true;
}

void AvoidManage::CalculateRanges(const std::vector<double>& start, const std::vector<double>& end,
                                  Eigen::Vector3d* const range, Eigen::Vector3d* const origin) 
{
  range->x() = std::abs(start[0] - end[0]) + 16;
  range->y() = std::abs(start[1] - end[1]) + 10;
  range->z() = std::abs(start[2] - end[2]) + 8;
  origin->x() = (start[0] + end[0]) / 2.0;
  origin->y() = (start[1] + end[1]) / 2.0;
  origin->z() = (start[2] + end[2]) / 2.0 + 2.0;
}

void AvoidManage::CalculateRanges(Eigen::Vector3d* const range, Eigen::Vector3d* const origin)
{
  range->x() = env_range[0];
  range->y() = env_range[1];
  range->z() = env_range[2];
  origin->x() = env_origin[0];
  origin->y() = env_origin[1];
  origin->z() = env_origin[2];
}

}
