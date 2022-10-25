#include <eigen3/Eigen/Eigen>
#include "quadrotor_common/quad_state_estimate.h"
#include "avoidlib/bridges/avoidbench_bridge.hpp"
#include <yaml-cpp/yaml.h>

namespace avoidmetrics {

class Mission {
public:
  Eigen::Vector3d start_point;
  Eigen::Vector3d end_point;
  int flight_numbers;
  bool finished;
  bool start_flag;
  bool stop_flag;
  double t_mission;
  double t_limit;
  std::vector<quadrotor_common::QuadStateEstimate> traj;
  double obs_width;
  double r_poisson;
  std::string pc_path;
  int collision_number;
  std::vector<float> cal_time;
  int trials;
  int mission_id, trial_id;

  Mission(){};
  Mission(const std::string &cfg_path, const avoidlib::mission_parameter &m_param, const int mission_id_);
  void getTrajectory(const quadrotor_common::QuadStateEstimate &state);
  ~Mission() {};
  void CollisionCount();
  void reset(const avoidlib::mission_parameter &m_param);

private:
  void MissionStopState();
};

}