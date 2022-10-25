#pragma once

#include <Eigen/Dense>
#include <rpg_quadrotor_msgs/TrajectoryPoint.h>
#include <ros/duration.h>

namespace quadrotor_common
{

struct TrajectoryPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryPoint();
  TrajectoryPoint(const rpg_quadrotor_msgs::TrajectoryPoint& trajectory_point_msg);
  virtual ~TrajectoryPoint();

  rpg_quadrotor_msgs::TrajectoryPoint toRosMessage() const;

  ros::Duration time_from_start;

  // Pose
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Linear derivatives
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;

  // Angular derivatives
  Eigen::Vector3d bodyrates;
  Eigen::Vector3d angular_acceleration;
  Eigen::Vector3d angular_jerk;
  Eigen::Vector3d angular_snap;

  // Heading angle with respect to world frame [rad]
  double heading;
  double heading_rate;
  double heading_acceleration;
};

} // namespace quadrotor_common
