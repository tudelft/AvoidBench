#include "rpg_single_board_io/voltage_reader.h"

#include <std_msgs/Float32.h>

namespace rpg_single_board_io
{

VoltageReader::VoltageReader()
{
  if (!loadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters",
              ros::this_node::getName().c_str());
    ros::shutdown();
  }

  if (adc_reader_.adcSetup(board_name_, adc_id_) < 0)
  {
    ROS_ERROR("[%s] Could not set up ADC", ros::this_node::getName().c_str());
    ros::shutdown();
  }

  // After successful setting up the ADC we can read the board specific
  // parameters
  max_adc_value_ = adc_reader_.getMaxAdcValue();
  max_adc_voltage_ = adc_reader_.getMaxAdcVoltage();

  voltage_pub_ = nh_.advertise<std_msgs::Float32>("voltage_reader/voltage", 1);

  read_battery_timer_ = nh_.createTimer(
      ros::Duration(1.0 / read_voltage_frequency_), &VoltageReader::readVoltage,
      this);
}

VoltageReader::~VoltageReader()
{
}

void VoltageReader::readVoltage(const ros::TimerEvent& time)
{
  unsigned int raw_adc_value;
  if (adc_reader_.adcReadRaw(&raw_adc_value) < 0)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] Could not read ADC",
                       ros::this_node::getName().c_str());
  }
  else
  {
    const double voltage = double(raw_adc_value) / max_adc_value_
        * max_adc_voltage_
        * (voltage_divider_upper_res_ + voltage_divider_lower_res_)
        / voltage_divider_lower_res_;

    std_msgs::Float32 voltage_msg;
    voltage_msg.data = voltage;
    voltage_pub_.publish(voltage_msg);
  }
}

bool VoltageReader::loadParameters()
{
  ros::NodeHandle pnh = ros::NodeHandle("~");

  if (!pnh.getParam("board_name", board_name_))
  {
    return false;
  }
  if (!pnh.getParam("adc_id", adc_id_))
  {
    return false;
  }
  if (!pnh.getParam("read_voltage_frequency", read_voltage_frequency_))
  {
    return false;
  }
  if (!pnh.getParam("voltage_divider_upper_res", voltage_divider_upper_res_))
  {
    return false;
  }
  if (!pnh.getParam("voltage_divider_lower_res", voltage_divider_lower_res_))
  {
    return false;
  }

  if (voltage_divider_upper_res_ <= 0.0 || voltage_divider_lower_res_ <= 0.0)
  {
    ROS_ERROR("Voltage divider resistor values must be strictly positive!");
    return false;
  }

  return true;
}

} // namespace rpg_single_board_io

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voltage_reader");
  rpg_single_board_io::VoltageReader voltage_reader;

  ros::spin();

  return 0;
}
