#pragma once

#include <memory>

#include "agilib/bridge/msg_encoding.hpp"

namespace agi {

class SbusEncoding : public MsgEncoding {
 public:
  SbusEncoding(const Quadrotor& quad, const ThrustMap& thrust_map,
               const Vector<3>& omega_max)
    : quad(quad), thrust_map(thrust_map), omega_max(omega_max) {}

  bool encode(const Command& command, const bool armed, char* const buffer,
              int* const length) const;

  void setVoltage(const Scalar voltage);

 private:
  static constexpr size_t N_CHANNELS = 16;
  static constexpr size_t CHANNEL_THROTTLE = 0;
  static constexpr size_t CHANNEL_OMEGAX = 1;
  static constexpr size_t CHANNEL_OMEGAY = 2;
  static constexpr size_t CHANNEL_OMEGAZ = 3;
  static constexpr size_t CHANNEL_ARMED = 4;
  static constexpr size_t CHANNEL_MODE = 5;

  static constexpr int SBUS_FRAME_LENGTH = 25;
  static constexpr char SBUS_HEADER_BYTE = 0x0F;
  static constexpr char SBUS_FOOTER_BYTE = 0x00;

  static constexpr size_t SBUS_MIN_VAL = 192;
  static constexpr size_t SBUS_MAX_VAL = 1792;
  static constexpr size_t SBUS_VAL_RANGE = SBUS_MAX_VAL - SBUS_MIN_VAL;

  const Quadrotor quad;
  const ThrustMap thrust_map;
  const Vector<3> omega_max;
  Scalar voltage{NAN};

  static void parseToBuffer(char* const buffer, const size_t* const channels,
                            const bool digital_channel_1 = false,
                            const bool digital_channel_2 = false,
                            const bool frame_lost = false,
                            const bool failsafe = false);
};

}  // namespace agi
