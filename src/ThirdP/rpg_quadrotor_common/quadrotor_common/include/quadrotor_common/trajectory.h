#pragma once

#include <list>

#include <rpg_quadrotor_msgs/Trajectory.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>

#include "quadrotor_common/trajectory_point.h"

namespace quadrotor_common
{

struct Trajectory
{
  Trajectory();
  Trajectory(const rpg_quadrotor_msgs::Trajectory& trajectory_msg);
  Trajectory(const quadrotor_common::TrajectoryPoint& point);
  virtual ~Trajectory();

  rpg_quadrotor_msgs::Trajectory toRosMessage() const;
  nav_msgs::Path toRosPath() const;
  quadrotor_common::TrajectoryPoint getStateAtTime(
    const ros::Duration& time_from_start) const;

  ros::Time timestamp;

  enum class TrajectoryType
  {
    UNDEFINED, GENERAL, ACCELERATION, JERK, SNAP
  } trajectory_type;

  std::list<quadrotor_common::TrajectoryPoint> points;
};

} // namespace quadrotor_common
