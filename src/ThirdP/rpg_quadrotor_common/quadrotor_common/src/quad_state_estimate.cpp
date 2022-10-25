#include "quadrotor_common/quad_state_estimate.h"

#include <string>

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

QuadStateEstimate::QuadStateEstimate() :
    timestamp(0), coordinate_frame(CoordinateFrame::INVALID),
        position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        bodyrates(Eigen::Vector3d::Zero())
{
}

QuadStateEstimate::QuadStateEstimate(
    const nav_msgs::Odometry& state_estimate_msg)
{
  timestamp = state_estimate_msg.header.stamp;
  coordinate_frame = CoordinateFrame::INVALID;
  if (state_estimate_msg.header.frame_id.compare("world") == 0)
  {
    coordinate_frame = CoordinateFrame::WORLD;
  }
  else if (state_estimate_msg.header.frame_id.compare("optitrack") == 0)
  {
    coordinate_frame = CoordinateFrame::OPTITRACK;
  }
  else if (state_estimate_msg.header.frame_id.compare("vision") == 0)
  {
    coordinate_frame = CoordinateFrame::VISION;
  }
  else if (state_estimate_msg.header.frame_id.compare("local") == 0)
  {
    coordinate_frame = CoordinateFrame::LOCAL;
  }
  position = geometryToEigen(state_estimate_msg.pose.pose.position);
  velocity = geometryToEigen(state_estimate_msg.twist.twist.linear);
  orientation = geometryToEigen(state_estimate_msg.pose.pose.orientation);
  bodyrates = geometryToEigen(state_estimate_msg.twist.twist.angular);
}

QuadStateEstimate::~QuadStateEstimate()
{
}

nav_msgs::Odometry QuadStateEstimate::toRosMessage() const
{
  nav_msgs::Odometry msg;

  msg.header.stamp = timestamp;
  switch (coordinate_frame)
  {
    case CoordinateFrame::WORLD:
      msg.header.frame_id = "world";
      break;
    case CoordinateFrame::OPTITRACK:
      msg.header.frame_id = "optitrack";
      break;
    case CoordinateFrame::VISION:
      msg.header.frame_id = "vision";
      break;
    case CoordinateFrame::LOCAL:
      msg.header.frame_id = "local";
      break;
    default:
      msg.header.frame_id = "invalid";
      break;
  }
  msg.child_frame_id = "body";
  msg.pose.pose.position = vectorToPoint(eigenToGeometry(position));
  msg.twist.twist.linear = eigenToGeometry(velocity);
  msg.pose.pose.orientation = eigenToGeometry(orientation);
  msg.twist.twist.angular = eigenToGeometry(bodyrates);

  return msg;
}

void QuadStateEstimate::transformVelocityToWorldFrame()
{
  velocity = orientation * velocity;
}

bool QuadStateEstimate::isValid() const
{
  if (coordinate_frame == CoordinateFrame::INVALID)
  {
    return false;
  }
  if (std::isnan(position.norm()))
  {
    return false;
  }
  if (std::isnan(velocity.norm()))
  {
    return false;
  }
  if (std::isnan(orientation.norm()))
  {
    return false;
  }
  if (std::isnan(bodyrates.norm()))
  {
    return false;
  }

  return true;
}

void QuadStateEstimate::setMatrix(Eigen::Ref<Eigen::Matrix<double,3,1>> pos, Eigen::Ref<Eigen::Matrix<double,4,1>> ori, double t)
{
  position = pos;
  orientation = ori;
  timestamp = ros::Time(t);
}

} // namespace quadrotor_common
