#include "agilib/bridge/laird/laird_encoding.hpp"

#include <fstream>
#include <iostream>

namespace agi {

LairdEncoding::LairdEncoding(const bool single_rotor_thrust)
  : single_rotor_thrust_(single_rotor_thrust) {}

bool LairdEncoding::encode(const Command& command, const bool armed,
                           char* const buffer, int* const length) const {
  if (length == nullptr || buffer == nullptr) return false;
  if (*length < BUFFER_SIZE) return false;

  if (!command.valid()) return false;

  bool is_single_rotor_thrust = false;
  if (single_rotor_thrust_) {
    if (command.isSingleRotorThrusts()) {
      is_single_rotor_thrust = true;
    } else {
      std::cout << "LairdEncoding single rotor not in the command" << std::endl;
      return false;
    }
  } else {
    if (command.isRatesThrust()) {
      is_single_rotor_thrust = false;
    } else {
      std::cout << "LairdEncoding rates thrust not in the command" << std::endl;
      return false;
    }
  }

  int idx = 0;
  memcpy(&buffer[idx], &command.t, SCALAR_SIZE);
  idx += SCALAR_SIZE;

  char flags = 0x00;
  flags += armed ? ARMBIT : 0x00;
  flags += !is_single_rotor_thrust ? 0x00 : MODEBIT;

  memcpy(&buffer[idx], &flags, FLAGS_SIZE);
  idx += FLAGS_SIZE;

  // Assigning values individually.
  // Could potentially copy the underlying array of the Eigen object.
  // However, this might not be contiguous in memory in the general case.
  // Therefore, we go with individual copy for the moment.
  if (is_single_rotor_thrust) {
    memcpy(&buffer[idx], &command.thrusts(0), SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.thrusts(1), SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.thrusts(2), SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.thrusts(3), SCALAR_SIZE);
    idx += SCALAR_SIZE;
  } else {
    memcpy(&buffer[idx], &command.collective_thrust, SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.omega(0), SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.omega(1), SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&buffer[idx], &command.omega(2), SCALAR_SIZE);
    idx += SCALAR_SIZE;
  }

  const CRC::Type crc_value = crc.compute(buffer, idx);
  memcpy(&buffer[idx], &crc_value, CRC::SIZE);
  idx += CRC::SIZE;

  if (idx != BUFFER_SIZE) return false;
  *length = idx;

  return true;
}

bool LairdEncoding::decode(const char* const buffer, const int length,
                           Command* const command, bool* const armed) const {
  if (buffer == nullptr || command == nullptr || armed == nullptr) return false;
  if (length < BUFFER_SIZE) return false;

  CRC::Type crc_received;
  memcpy(&crc_received, &buffer[BUFFER_SIZE - CRC::SIZE], CRC::SIZE);
  const CRC::Type crc_value = crc.compute(buffer, MSG_SIZE);

  if (crc_value != crc_received) return false;

  int idx = 0;
  memcpy(&command->t, &buffer[idx], SCALAR_SIZE);
  idx += SCALAR_SIZE;

  char flags;
  memcpy(&flags, &buffer[idx], FLAGS_SIZE);
  idx += FLAGS_SIZE;

  *armed = flags & ARMBIT;
  const bool is_single_rotor_thrusts = flags & MODEBIT;

  if (is_single_rotor_thrusts) {
    memcpy(&command->thrusts(0), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->thrusts(1), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->thrusts(2), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->thrusts(3), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    command->collective_thrust = NAN;
    command->omega.setConstant(NAN);
  } else {
    memcpy(&command->collective_thrust, &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->omega(0), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->omega(1), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    memcpy(&command->omega(2), &buffer[idx], SCALAR_SIZE);
    idx += SCALAR_SIZE;
    command->thrusts.setConstant(NAN);
  }

  return true;
}


}  // namespace agi
