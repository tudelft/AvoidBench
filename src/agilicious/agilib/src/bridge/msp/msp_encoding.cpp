#include "agilib/bridge/msp/msp_encoding.hpp"

#include "agilib/bridge/msp/msp_msgs_defs.hpp"

namespace agi {

MspEncoding::MspEncoding(const ThrustMap& thrust_map)
  : thrust_map_(thrust_map), motor_packet_(16) {
  motor_packet_.code() = msp::SET_MOTOR;
  motor_packet_.direction() = msp::SEND;
}

bool MspEncoding::encode(const Command& command, const bool armed,
                         char* const buffer, int* const length) const {
  if (buffer == nullptr) return false;
  const bool armed_and_valid = armed && command.isSingleRotorThrusts();

  const Scalar voltage = 16.0;
  for (int i = 0; i < 4; ++i) {
    const Scalar T = thrust_map_.map(4.0 * command.thrusts(i), voltage);
    motor_packet_.payload<uint16_t>()[motor_order[i]] =
      armed_and_valid ? T : 1000;
  }
  motor_packet_.finalize();

  *length = motor_packet_.message_size();
  motor_packet_.copy_to_buffer(buffer);

  return true;
}

bool MspEncoding::decode(const char* const buffer, const int length,
                         Command* const command, bool* const armed) const {
  return false;
}

bool MspEncoding::decode(const char* const buffer, const int length) const {
  if (length < msp::MIN_LENGTH || length > SerialPort::BUFFER_SIZE)
    return false;

  bool valid = true;
  const char* idx = std::find(buffer, buffer + length, '$');
  const int i = idx - buffer;
  if (i > length - msp::MIN_LENGTH) {
    valid = false;
    return false;
  }
  // Message format is which is checked is $M<
  valid &= (buffer[i + 1] == 'M');
  valid &= (buffer[i + 2] == msp::RECV);

  // Next char [3] is payload length
  const uint8_t payload_length = buffer[i + 3];
  const uint8_t remaining_length =
    length - i - payload_length - msp::MIN_LENGTH;
  if (!valid || payload_length > msp::MAX_PAYLOAD || remaining_length < 0) {
    std::cout << "No Remaining Length" << std::endl;
    return false;
  }

  MspPacket pkt(payload_length);
  pkt.direction() = msp::RECV;

  // Message code is fourth byte of the message and payload starts from 5th on
  pkt.code() = buffer[i + 4];
  memcpy(pkt.payload(), &buffer[i + 5], pkt.payload_length());
  pkt.finalize();

  if (pkt.checksum() != buffer[i + msp::MIN_LENGTH + payload_length - 1]) {
    std::cout << "Checksums do not match" << std::endl;
    return false;
  }

  return valid;
}

}  // namespace agi
