
#pragma once

#include <memory>
#include <fstream>
// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelState.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include "quadrotor_common/quad_state_estimate.h"
#include "quadrotor_common/trajectory.h"
#include <rpg_quadrotor_msgs/AutopilotFeedback.h>

// avoidlib
#include "avoidlib/bridges/unity_bridge.hpp"
#include "avoidlib/bridges/avoidbench_bridge.hpp"
#include "avoidlib/common/quad_state.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/objects/quadrotor.hpp"
#include "avoidlib/sensors/rgb_camera.hpp"
#include "Metrics.h"
#include "avoid_msgs/TaskState.h"
#include "avoid_msgs/PointCloudInfo.h"
#include <ros/package.h>

using namespace avoidlib;

namespace avoid_manage {

class AvoidManage {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AvoidManage(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~AvoidManage();

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  enum Mission_state
  {
    PREPARING = 0,
    UNITYSETTING = 1,
    SENTMISSION = 2,
    WAITMISSION = 3,
    MISSIONPROCESS = 4,
    GAZEBOSETTING = 5
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher

  // subscriber
  ros::Subscriber sub_state_est_;
  ros::Subscriber odo_sub_;
  ros::Subscriber mission_start_sub_;

  ros::Subscriber iter_time_sub_;
  ros::Subscriber save_pc_sub_;
  ros::Subscriber training_period_sub;

  ros::Publisher start_pub_;
  ros::Publisher force_hover_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher off_pub_;
  ros::Publisher arm_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher task_state_pub_;
  ros::Publisher avoid_metrics_pub_;
  ros::Publisher collision_info_pub_;

  ros::ServiceClient gazebo_model_srv_;

  // main loop timer
  ros::Timer metrics_loop_;
  ros::Timer task_loop_;
  ros::Timer mission_state_loop_;
  ros::Timer render_loop_;

  std::shared_ptr<AvoidbenchBridge> avoidbench_bridge;
  std::shared_ptr<avoidmetrics::Metrics> metrics;

  image_transport::Publisher left_pub;
  image_transport::Publisher right_pub;
  image_transport::Publisher depth_pub;
  ros::Publisher depth_cam_info_pub;

  int mission_number;
  bool save_data_mode{false};
  bool start_ok;
  bool get_seed{false};
  bool if_traning_period{false};
  bool last_collision_state{true};
  bool collision_state{true};
  bool collision_happen{false};
  bool get_new_mission{true};
  nav_msgs::Odometry drone_pose;
  int mission_id;
  int trial_id;
  std::string cfg_;
  quadrotor_common::QuadStateEstimate received_state_est_;
  std::vector<float> iter_times;
  ros::Time mission_start_time;
  std::string quad_name;

  // Flightmare(Unity3D)
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_init_{false};
  bool unity_render_{false};
  bool first_env_{false};
  bool if_update_map{true};
  RenderMessage_t unity_output_;

  // auxiliary variables
  float main_loop_freq_{30.0};
  float task_loop_freq_{50.0};

  double start_time;
  avoidlib::mission_parameter MissionParam;
  bool mission_start_flag_;
  Mission_state mission_state;

  //camera info
  double fov;
  int width;
  int height;
  double baseline;
  bool perform_sgm_;
  std::vector<double> env_range, env_origin;

  std::ofstream debug_out;
  
  void getMissionParam(avoidlib::mission_parameter* const m_param, const int &m_id);
  void resetGazebo(const Eigen::Vector3d &pos, const double yaw);
  bool TakeoffSuccess();
  void CalculateRanges(const std::vector<double>& start, const std::vector<double>& end,
                      Eigen::Vector3d* const range, Eigen::Vector3d* const origin);
  void CalculateRanges(Eigen::Vector3d* const range, Eigen::Vector3d* const origin);
  Eigen::Quaterniond ypr2quat(double yaw, double pitch, double roll);
  void MetricsThread();

  // callbacks
  void RenderingCallback(const ros::TimerEvent &event);
  void MetricsLoopCallback(const ros::TimerEvent &event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void MissionFlagCallback(const std_msgs::Empty::ConstPtr &msg);
  void IterTimeCallback(const std_msgs::Float32::ConstPtr &msg);
  void MissionCallback(const ros::TimerEvent &event);
  void MissionStateLoopCallback(const ros::TimerEvent &event);
  void SavePointcloudCallback(const avoid_msgs::PointCloudInfoConstPtr &msg);
  void TrainingStateCallback(const std_msgs::BoolConstPtr &msg);
};
}  // namespace avoid_manage