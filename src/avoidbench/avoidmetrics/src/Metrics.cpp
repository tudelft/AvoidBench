#include "Metrics.h"

namespace avoidmetrics
{
  Metrics::Metrics(const std::string &cfg_path)
  : task_finished(false)
  {
    YAML::Node cfg_ = YAML::LoadFile(cfg_path);
    if (!loadParameters(cfg_)) {
      std::cout<<"load parameter failed 2"<<std::endl;
    }

    env_ptr_ = std::make_shared<Environment>(cfg_);
    astar_ptr_ = std::make_shared<AStarFindPath>();
  }

  double Metrics::CalOptimalityFactor(const double &dis, 
                                      const std::shared_ptr<Mission> mission, double &trav_dis)
  {
    trav_dis = 0;
    if(mission->traj.size() == 0) {
      trav_dis = 0;
      return 1.0;
    }
    
    std::vector<quadrotor_common::QuadStateEstimate> traj = mission->traj;
    double d_opt = (traj.back().position - traj.front().position).norm();
    for(int i=0; i<(traj.size()-1); i++)
    {
      trav_dis += (traj[i+1].position - traj[i].position).norm();
    }
    if(!mission->finished) return 1.0;
    if((trav_dis - dis)>0)
      return (trav_dis - dis) / dis;
    else
      return (trav_dis - d_opt) / d_opt;
  }

  void Metrics::CalOptimalDis(double* const dis, const std::vector<Eigen::Vector3d> &path)
  {
    double distance = 0;
    for(int i=0; i<path.size()-1; i++)
    {
      distance = distance + (path[i+1] - path[i]).norm();
    }
    *dis = distance;
  }

  double Metrics::CalAverageGoalVelocity(const double &dis, const double &t_m)
  {
    return dis / t_m;
  }

  double Metrics::CalMissionProgress(const std::shared_ptr<Mission> mission)
  {
    double progress;
    if(mission->finished)
      progress = 1.0;
    else
    {
      collision_times++;
      Eigen::Vector3d a = mission->end_point - mission->start_point;
      Eigen::Vector3d b = mission->traj.back().position - mission->end_point;
      progress = 1 - abs(a.dot(b)/(a.norm()*a.norm()));
    }
    return progress;
  }

  double Metrics::CalRelativeGapSize(const double &width, const double &radius)
  {
    double relative_gap_size;
    relative_gap_size = (radius-width) / d_drone;
    return relative_gap_size;
  }

  double Metrics::CalProcessingTime(const std::vector<float> &time)
  {
    double average = 0;
    for(int i=0; i<time.size(); i++)
    {
      average += time[i];
    }
      
    return average/time.size();
  }

  void Metrics::setMissions(const std::shared_ptr<Mission> mission)
  {
    mission_.push_back(mission);
  }

  avoid_msgs::Metrics Metrics::getMetricsMsg()
  {
    return metrics_msg;
  }

  void Metrics::setTaskFinishFlag(const bool &finished)
  {
    task_finished = finished;
  }

  double Metrics::CalEnergyCost(const double &dis, const std::vector<quadrotor_common::QuadStateEstimate> traj)
  {
    double energy = 0;
    for(int i=0; i<traj.size() - 2; i++)
    {
      if((traj[i+1].timestamp - traj[i].timestamp).toSec() <= 1e-5) continue;
      Eigen::Vector3d ai = (traj[i+1].velocity - traj[i].velocity) / (traj[i+1].timestamp - traj[i].timestamp).toSec();
      while((traj[i+2].timestamp - traj[i+1].timestamp).toSec() <= 1e-5) i++;
      Eigen::Vector3d aj = (traj[i+2].velocity - traj[i+1].velocity) / (traj[i+2].timestamp - traj[i+1].timestamp).toSec();
      Eigen::Vector3d jerk = aj - ai;
      energy = energy + jerk.norm();
    }
    return energy / dis;
  }

  bool Metrics::run()
  {
    avoid_msgs::Factor factor;
    while(!task_finished)
    {
      if(!mission_.empty())
      {
        std::cout<<"mission number: "<<mission_.size()<<std::endl;
        std::shared_ptr<Mission> mission = mission_.front();
        double dis, trav_dis;

        if(mission->trial_id == 0)
        {
          //just need to be calculated once
          std::string pc_path = getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/point_clouds_data/") + 
                                mission->pc_path + std::string(".ply");
          env_ptr_->readPointCloud(pc_path);
          astar_ptr_->setMap(env_ptr_);
          factor.traversability = env_ptr_->getTraversability();
          factor.traversability = factor.traversability / d_drone;
          factor.relative_gap_size = CalRelativeGapSize(mission->obs_width, mission->r_poisson);
          factor.optimality_factor.clear();
          factor.average_goal_velocity.clear();
          factor.mission_progress.clear();
          factor.processing_time.clear();
          factor.collision_number.clear();
          factor.energy_cost.clear();
        }
        astar_ptr_->toFindPath(mission->start_point, mission->end_point);
        CalOptimalDis(&dis, astar_ptr_->getBestPath());
        factor.optimality_factor.push_back(CalOptimalityFactor(dis, mission, trav_dis));
        factor.average_goal_velocity.push_back(CalAverageGoalVelocity(dis, mission->t_mission));
        factor.mission_progress.push_back(CalMissionProgress(mission));
        factor.processing_time.push_back(CalProcessingTime(mission->cal_time));
        factor.collision_number.push_back(mission->collision_number);
        factor.energy_cost.push_back(CalEnergyCost(dis, mission->traj));
        std::cout<<"distance: "<<dis<<std::endl;
        std::cout<<"traversability: "<<factor.traversability<<std::endl;
        std::cout<<"relative_gap_size: "<<factor.relative_gap_size<<std::endl;
        std::cout<<"optimality_factor: "<<factor.optimality_factor.back()<<std::endl;
        std::cout<<"average_goal_velocity: "<<factor.average_goal_velocity.back()<<std::endl;
        std::cout<<"mission_progress: "<<factor.mission_progress.back()<<std::endl;
        std::cout<<"processing_time: "<<factor.processing_time.back()<<std::endl;
        std::cout<<"collision_number: "<<factor.collision_number.back()<<std::endl;
        std::cout<<"energy_cost: "<<factor.energy_cost.back()<<std::endl;

        metrics_out.open(getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/data/metrics_out.txt"), 
                          std::ios::out | std::ios::app);
        std::string out_data;
        out_data = std::to_string(dis) + " " + std::to_string(trav_dis) + " " + 
                    std::to_string(factor.traversability) + " " +
                    std::to_string(factor.relative_gap_size) + " " +
                    std::to_string(factor.optimality_factor.back()) + " " +
                    std::to_string(factor.average_goal_velocity.back()) + " " +
                    std::to_string(factor.mission_progress.back()) + " " +
                    std::to_string(factor.processing_time.back()) + " " +
                    std::to_string(factor.collision_number.back()) + " " +
                    std::to_string(factor.energy_cost.back()) + "\n";
        metrics_out << out_data;
        metrics_out.close();

        if(mission->trial_id == (mission->trials-1))
        {
          metrics_msg.factors.push_back(factor);
        }
        mission_.pop_front();
      }
      usleep(0.05*1e6);
    }
    collision_percent = (double) collision_times / Trials;
    metrics_msg.collision_percent = collision_percent;
    return true;
  }

  bool Metrics::loadParameters(const YAML::Node &cfg)
  {
    if (cfg["drone"] && cfg["unity"])
    {
      double width = cfg["drone"]["width"].as<double>();
      double length = cfg["drone"]["length"].as<double>();
      d_drone = sqrt(width*width + length*length);
      return true;
    }
    return false;
  }
}