#include "Mission.h"
namespace avoidmetrics {
  Mission::Mission(const std::string &cfg_path, const avoidlib::mission_parameter &m_param, const int mission_id_)
  : start_flag(false),
    stop_flag(false),
    finished(false),
    t_limit(80.0), 
    trial_id(0),
    mission_id(mission_id_)
  {
    YAML::Node cfg_ = YAML::LoadFile(cfg_path);
    std::vector<double> min_object_scale = cfg_["unity"]["min_object_scale"].as<std::vector<double>>();
    std::vector<double> max_object_scale = cfg_["unity"]["max_object_scale"].as<std::vector<double>>();
    std::vector<double> basic_size = cfg_["unity"]["basic_size"].as<std::vector<double>>();
    obs_width = basic_size[0]*sqrt(pow((min_object_scale[0]+max_object_scale[0])/2.0, 2) + 
                              pow((min_object_scale[0]+max_object_scale[0])/2.0, 2));

    r_poisson = m_param.m_radius;
    start_point.x() = m_param.m_start_point[0];
    start_point.y() = m_param.m_start_point[1];
    start_point.z() = m_param.m_start_point[2];
    end_point.x() = m_param.m_end_point[0];
    end_point.y() = m_param.m_end_point[1];
    end_point.z() = m_param.m_end_point[2];
    pc_path = m_param.m_pc_file_name;
    collision_number = 0;
    trials = m_param.trials;
  }

  void Mission::getTrajectory(const quadrotor_common::QuadStateEstimate &state)
  {
    traj.push_back(state);
    t_mission = (state.timestamp - traj.front().timestamp).toSec();
    MissionStopState();
  }

  void Mission::MissionStopState()
  {
    if(traj.size() != 0)
    {
      quadrotor_common::QuadStateEstimate last_point = traj.back();
      if(sqrt(pow((last_point.position.x()-end_point.x()),2) + pow((last_point.position.y()-end_point.y()),2))<0.8)
      {
        finished = true;
        // stop_flag = true;
      }
      if(t_mission >= t_limit)
      {
        finished = false;
        stop_flag = true;
      }
    }
  }

  void Mission::CollisionCount()
  {
    collision_number++;
    // if(collision_number>2)
    // {
    //   finished = false;
    //   stop_flag = true;
    // }
  }

  void Mission::reset(const avoidlib::mission_parameter &m_param)
  {
    start_flag = false;
    stop_flag = false;
    finished = false;
    collision_number = 0;
    traj.clear();
    t_mission = 0;
    trial_id++;
    start_point.x() = m_param.m_start_point[0];
    start_point.y() = m_param.m_start_point[1];
    start_point.z() = m_param.m_start_point[2];
    end_point.x() = m_param.m_end_point[0];
    end_point.y() = m_param.m_end_point[1];
    end_point.z() = m_param.m_end_point[2];
  }
}