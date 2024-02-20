#pragma once

#include <memory>

#include "agilib/bridge/msg_encoding.hpp"
#include "agilib/serial/crc.hpp"

namespace agi {

class LairdEncoding : public MsgEncoding {
 public:
  LairdEncoding(const bool single_rotor_thrust = false);
  virtual bool encode(const Command& command, const bool armed,
                      char* const buffer, int* const length) const override;

  virtual bool decode(const char* const buffer, const int length,
                      Command* const command, bool* const armed) const override;

  /* Length of the message:
   *  1 double for time [8 byte]
   *  4 double for command values [4 * 8 = 32 byte]
   *  1 byte for flags [1 byte]
   *  2 bytes CRC16
   *  = (1 + 4) * 8 + 1 + 2 = 43 byte
   **/
  static constexpr int SCALAR_SIZE = sizeof(Scalar);
  static constexpr int FLAGS_SIZE = 1;
  static constexpr int NVALUES = 4;
  static constexpr int MSG_SIZE = (1 + NVALUES) * SCALAR_SIZE + FLAGS_SIZE;
  static constexpr int BUFFER_SIZE = MSG_SIZE + CRC16::SIZE;

  static constexpr char ARMBIT = 1 << 0;
  static constexpr char MODEBIT = 1 << 1;

 protected:
  using CRC = CRC16;
  static constexpr CRC crc{};

 private:
  const bool single_rotor_thrust_;
};

}  // namespace agi
