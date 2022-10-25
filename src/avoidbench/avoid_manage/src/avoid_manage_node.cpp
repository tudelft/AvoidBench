#include <ros/ros.h>

#include "avoid_manage.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "avoid_manage_node");
  avoid_manage::AvoidManage pilot(ros::NodeHandle(), ros::NodeHandle("~"));

  // spin the ros
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}
