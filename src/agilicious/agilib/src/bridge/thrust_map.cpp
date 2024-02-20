#include "agilib/bridge/thrust_map.hpp"

#include <fstream>
#include <iostream>

#include "agilib/base/parameter_base.hpp"

namespace agi {


bool ThrustMap::load(const fs::path& file) {
  std::ifstream csvFile;
  csvFile.open(file.c_str());

  if (!csvFile.is_open())
    throw ParameterException("Could not open thrust map file!");

  std::string line;

  // First line with general parameter
  if (!getline(csvFile, line))
    throw ParameterException("Could not read first line of thrust map csv!");

  std::istringstream ss(line);
  std::string num;

  if (!getline(ss, num, ','))
    throw ParameterException("Error in thrust map csv!");
  const int map_size = stoi(num);

  if (!getline(ss, num, ','))
    throw ParameterException("Error in thrust map csv!");
  thrust_min_ = stod(num);

  if (!getline(ss, num, ','))
    throw ParameterException("Error in thrust map csv!");
  thrust_max_ = stod(num);

  if (!getline(ss, num, ','))
    throw ParameterException("Error in thrust map csv!");
  voltage_min_ = stod(num);

  if (!getline(ss, num, ','))
    throw ParameterException("Error in thrust map csv!");
  voltage_max_ = stod(num);

  if ((voltage_max_ - voltage_min_) < 0.5) {
    voltage_max_ += 0.5;
  }

  map_ = Matrix<>::Zero(map_size, map_size);

  for (int row = 0; row < map_.rows(); ++row) {
    if (!getline(csvFile, line))
      throw ParameterException("Thrust map csv has wrong size!");
    ss = std::istringstream(line);

    for (int col = 0; col < map_.cols(); ++col) {
      if (!getline(ss, num, ','))
        throw ParameterException("Thrust map csv has wrong size!");
      map_(row, col) = stod(num);
    }
  }

  return true;
}

bool ThrustMap::valid() {
  return std::isfinite(voltage_min_) && std::isfinite(voltage_max_) &&
         std::isfinite(thrust_min_) && std::isfinite(thrust_max_) &&
         map_.allFinite();
}

Scalar ThrustMap::map(const Scalar thrust, const Scalar voltage) const {
  const int i_thrust{
    std::clamp<int>(((int)(Scalar)map_.cols() * (thrust - thrust_min_) /
                     (thrust_max_ - thrust_min_)),
                    0, map_.cols() - 2)};
  const int i_voltage{
    std::clamp<int>(((int)(Scalar)map_.rows() * (voltage - voltage_min_) /
                     (voltage_max_ - voltage_min_)),
                    0, map_.rows() - 2)};

  const Matrix<2, 2> W = map_.block<2, 2>(i_voltage, i_thrust);

  const Scalar delta_thrust = (thrust_max_ - thrust_min_) / (map_.cols() - 1);
  const Scalar x0_thrust = i_thrust * delta_thrust + thrust_min_;
  const Scalar delta_voltage =
    (voltage_max_ - voltage_min_) / (map_.rows() - 1);
  const Scalar x0_voltage = i_voltage * delta_voltage + voltage_min_;
  const Scalar x_thrust = (thrust - x0_thrust) / (delta_thrust);
  const Scalar x_voltage = (voltage - x0_voltage) / (delta_voltage);


  const Matrix<2, 1> xv_thrust{1.0 - x_thrust, x_thrust};
  const Matrix<1, 2> xv_voltage{1.0 - x_voltage, x_voltage};


  const Scalar command = xv_voltage * W * xv_thrust;

  return command;
}

}  // namespace agi
