#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>

#include <ros/ros.h>

#include "rpg_single_board_io/gpio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_interrupt_example");
  ros::NodeHandle nh;

  const unsigned int num_gpio = 25; // Example GPIO for Odroid XU4
  //const unsigned int num_gpio = 200; // Example GPIO for Odroid U3

  rpg_single_board_io::GPIO gpio(num_gpio,
                                 rpg_single_board_io::GpioEdge::Rising);

  if (!gpio.gpioIsOpen())
  {
    ROS_ERROR("[%s] Error while opening GPIO, exiting",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }

  ros::Time time_gpio_interrupt_detected;

  struct pollfd fdset[1];
  memset((void*)fdset, 0, sizeof(fdset));
  fdset[0].fd = gpio.gpioGetFd();
  fdset[0].events = POLLPRI;

  int rc, len;
  int poll_timeout = 5000;
  int nfds = 1;

  while (nh.ok())
  {
    rc = poll(fdset, nfds, poll_timeout);

    if (rc < 0)
    {
      ROS_WARN("[%s] Poll failed!", ros::this_node::getName().c_str());
    }
    if (rc == 0)
    {
      ROS_WARN("[%s] Poll Timeout!", ros::this_node::getName().c_str());
    }

    if (fdset[0].revents & POLLPRI)
    {
      // Save time at which interrupt was detected
      time_gpio_interrupt_detected = ros::Time::now();

      // Get to start of file and read it
      // This is necessary for the next interrupt event to be detected properly
      lseek(fdset[0].fd, 0, SEEK_SET);
      const int max_buf_length = 64;
      char buf[max_buf_length];
      len = read(fdset[0].fd, buf, max_buf_length);

      ROS_INFO("[%s] GPIO Interrupt detected.",
               ros::this_node::getName().c_str());
    }
  }

  return 0;
}
