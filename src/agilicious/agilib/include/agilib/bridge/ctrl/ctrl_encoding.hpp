#pragma once

#include <mutex>

#include "agilib/bridge/ctrl/ctrl_msgs_defs.hpp"
#include "agilib/bridge/msg_encoding.hpp"
#include "agilib/serial/crc.hpp"
#include "agilib/types/feedback.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

class CtrlEncoding : public MsgEncoding {
 public:
  CtrlEncoding(const Quadrotor& quad, const ThrustMap& thrust_map,
               const ctrl::CTRLMODE mode = ctrl::CTRLMODE::ROTOR_THROTTLE,
               bool command_throttle_direct = false,
               const Scalar thrust_coeff = 1.56e-6);

  virtual bool encode(const Command& command, const bool armed,
                      char* const buffer, int* const length) const override;

  bool decodeFeedback(const char* const buffer, const int length,
                      Feedback* const feedback = nullptr);

  bool encodeMessage(const ctrl::serial_message_t& message, char* const buffer,
                     int* const length) const;
  bool decodeMessage(const char* const buffer, const int length,
                     ctrl::serial_message_t* const message) const;

  bool encodeContainer(const ctrl::serial_container_t& container,
                       char* const buffer, int* const length) const;
  bool decodeContainer(const char* const buffer, const int length,
                       ctrl::serial_container_t* const container) const;

  const Feedback& feedback() const { return feedback_; }
  bool getFeedback(Feedback* const feedback = nullptr) {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    *feedback = feedback_;
    return feedback->valid();
  };

  Scalar getVoltage() { return voltage_; }

 private:
  // Helper functions
  static int getPayloadLength(const ctrl::CTRLMODE mode);
  static int getPayloadLength(const uint8_t mode);
  static int getMsgLength(const ctrl::CTRLMODE mode);
  static int getMsgLength(const uint8_t mode);

  Logger logger_{"CtrlEncoding"};
  using CRC = CRC16;
  static constexpr CRC crc_{};

  static constexpr char DELIMITER = ctrl::SERIAL_CONTAINER_DELIMITER;

  const Quadrotor quad_;
  const ThrustMap thrust_map_;
  const ctrl::CTRLMODE mode_;
  const bool command_throttle_direct_;
  const Scalar thrust_coeff_;
  mutable uint64_t time_ns_init_{0};
  Feedback feedback_;
  std::mutex feedback_mtx_;
  const Scalar voltage_;
};

}  // namespace agi
