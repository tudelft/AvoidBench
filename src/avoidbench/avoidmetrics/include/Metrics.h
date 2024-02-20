#pragma once
#include "Mission.h"
#include "environment.h"
#include "astar_path_finding.h"
#include "avoidlib/common/types.hpp"
#include "avoidlib/common/math.hpp"
#include "avoid_msgs/Factor.h"
#include "avoid_msgs/Metrics.h"
#include <memory>
#include <fstream>
namespace avoidmetrics
{

struct Metrics_msg {
double traversability;
double relative_gap_size;
std::vector<double> optimality_factor;
std::vector<double> average_goal_velocity;
std::vector<double> mission_progress;
std::vector<double> processing_time;
std::vector<int> collision_number;
std::vector<double> energy_cost;
};

class Metrics {
public:
  Metrics(const std::string &cfg_path);
  ~Metrics() {};

  void setMissions(const std::shared_ptr<Mission> mission);
  bool run();
  void setTaskFinishFlag(const bool &finished);
  avoid_msgs::Metrics getMetricsMsg();
  bool task_finished;

private:
  int collision_times;
  double collision_percent;
  int Trials;
  double d_drone;
  std::ofstream metrics_out;

  avoid_msgs::Metrics metrics_msg;
  std::list<std::shared_ptr<Mission>> mission_;
  std::shared_ptr<Environment> env_ptr_;
  std::shared_ptr<AStarFindPath> astar_ptr_;
  void setState(const quadrotor_common::QuadStateEstimate& state);
  void CalOptimalDis(double* const dis, const std::vector<Eigen::Vector3d> &path);
  double CalOptimalityFactor(const double& dis,
                              const std::shared_ptr<Mission> mission, double &trav_dis);
  double CalAverageGoalVelocity(const double &dis, const double &t_m);
  double CalMissionProgress(const std::shared_ptr<Mission> mission);
  double CalRelativeGapSize(const double &width, const double &radius);
  double CalProcessingTime(const std::vector<float> &time);
  double CalEnergyCost(const double &dis, const std::vector<quadrotor_common::QuadStateEstimate> traj);
  bool loadParameters(const YAML::Node &cfg);
};
}
