#pragma once

#include <algorithm>
#include <cstring>
#include <iterator>

#include "agilib/bridge/msg_encoding.hpp"
#include "agilib/bridge/msp/msp_packet.hpp"
#include "agilib/serial/serial.hpp"
#include "agilib/types/feedback.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

class MspEncoding : public MsgEncoding {
 public:
  MspEncoding(const ThrustMap& thrust_map);

  virtual bool encode(const Command& command, const bool armed,
                      char* const buffer, int* const length) const override;

  virtual bool decode(const char* const buffer, const int length,
                      Command* const command, bool* const armed) const override;

  virtual bool decode(const char* const buffer, const int length) const;

  const Feedback& feedback() const { return feedback_; }

 private:
  Logger logger_{"MspEncoding"};

  const ThrustMap thrust_map_;
  mutable MspPacket motor_packet_;

  static constexpr int motor_order[4] = {1, 2, 0, 3};

  Feedback feedback_;
};

}  // namespace agi
