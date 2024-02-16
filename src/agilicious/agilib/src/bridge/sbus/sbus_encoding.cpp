#include "agilib/bridge/sbus/sbus_encoding.hpp"

namespace agi {

bool SbusEncoding::encode(const Command& command, const bool armed,
                          char* const buffer, int* const length) const {
  if (length == nullptr || buffer == nullptr) return false;
  if (*length < SBUS_FRAME_LENGTH) return false;

  if (!std::isfinite(voltage)) return false;

  std::fill(buffer, buffer + SBUS_FRAME_LENGTH, 0x00);

  // Map a desired thrust in Newton to an SBUS command.
  const Scalar sbus_thrust =
    thrust_map.map(command.collective_thrust * quad.m_, voltage);

  // Catch mapping error for zero thrust.
  const Scalar sbus_thrust_apply =
    std::max(command.collective_thrust > 0.0 ? sbus_thrust : 0.0, 0.0);

  // Convert to unsigned int and clamp in valid SBUS range.
  const size_t clamped_sbus_thrust =
    std::clamp((size_t)sbus_thrust_apply, SBUS_MIN_VAL, SBUS_MAX_VAL);

  // Scale rate to (0, 1) and map into SBUS range below.
  const Array<3, 1> scale = Array<3, 1>::Ones() / (2.0 * omega_max.array());
  const Vector<3> sbus_omega{command.omega.x(), command.omega.y(),
                             -command.omega.z()};
  const Vector<3> normal_rates =
    scale *
    (sbus_omega.cwiseMax(-omega_max).cwiseMin(omega_max) + omega_max).array();

  // Set channels.
  size_t channels[N_CHANNELS] = {0};
  channels[CHANNEL_THROTTLE] = clamped_sbus_thrust;
  channels[CHANNEL_OMEGAX] =
    SBUS_MIN_VAL + (size_t)(SBUS_VAL_RANGE * normal_rates.x());
  channels[CHANNEL_OMEGAY] =
    SBUS_MIN_VAL + (size_t)(SBUS_VAL_RANGE * normal_rates.y());
  channels[CHANNEL_OMEGAZ] =
    SBUS_MIN_VAL + (size_t)(SBUS_VAL_RANGE * normal_rates.z());
  channels[CHANNEL_ARMED] = armed ? SBUS_MAX_VAL : SBUS_MIN_VAL;
  channels[CHANNEL_MODE] = SBUS_MAX_VAL;

  // Parse into serial buffer according to SBUS protocol.
  parseToBuffer(buffer, channels);

  *length = SBUS_FRAME_LENGTH;
  return true;
}

void SbusEncoding::parseToBuffer(char* const buffer,
                                 const size_t* const channels,
                                 const bool digital_channel_1,
                                 const bool digital_channel_2,
                                 const bool frame_lost, const bool failsafe) {
  if (buffer == nullptr) return;  // TODO: Handle exception

  buffer[0] = SBUS_HEADER_BYTE;

  // 16 channels of 11 bit data
  buffer[1] = (uint8_t)((channels[0] & 0x07FF));
  buffer[2] =
    (uint8_t)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
  buffer[3] =
    (uint8_t)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
  buffer[4] = (uint8_t)((channels[2] & 0x07FF) >> 2);
  buffer[5] =
    (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
  buffer[6] =
    (uint8_t)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
  buffer[7] =
    (uint8_t)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
  buffer[8] = (uint8_t)((channels[5] & 0x07FF) >> 1);
  buffer[9] =
    (uint8_t)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
  buffer[10] =
    (uint8_t)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
  buffer[11] = (uint8_t)((channels[7] & 0x07FF) >> 3);
  buffer[12] = (uint8_t)((channels[8] & 0x07FF));
  buffer[13] =
    (uint8_t)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
  buffer[14] =
    (uint8_t)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
  buffer[15] = (uint8_t)((channels[10] & 0x07FF) >> 2);
  buffer[16] =
    (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
  buffer[17] =
    (uint8_t)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
  buffer[18] =
    (uint8_t)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
  buffer[19] = (uint8_t)((channels[13] & 0x07FF) >> 1);
  buffer[20] =
    (uint8_t)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
  buffer[21] =
    (uint8_t)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
  buffer[22] = (uint8_t)((channels[15] & 0x07FF) >> 3);

  buffer[23] = 0x00;
  if (digital_channel_1) {
    buffer[23] |= 0x01;
  }
  if (digital_channel_2) {
    buffer[23] |= 0x02;
  }
  if (frame_lost) {
    buffer[23] |= 0x04;
  }
  if (failsafe) {
    buffer[23] |= 0x08;
  }

  buffer[24] = SBUS_FOOTER_BYTE;
}

void SbusEncoding::setVoltage(const Scalar v) { voltage = v; }

}  // namespace agi
