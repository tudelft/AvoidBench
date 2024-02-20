#pragma once

#include "agilib/bridge/msp/msp_bridge_params.hpp"
#include "agilib/bridge/msp/msp_encoding.hpp"
#include "agilib/bridge/serial_bridge.hpp"
#include "agilib/types/feedback.hpp"

namespace agi {

class MspBridge : public SerialBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MspBridge(const Quadrotor& quad, const MspParams& params,
            const TimeFunction time_function);

  virtual bool getFeedback(Feedback* const feedback = nullptr) override;

 protected:
  using SerialBridge::sendCommand;
  virtual bool sendCommand(const Command& command, const bool active) override;
  void receiveCallback(const char* const buffer, const int length);

  MspEncoding encoding_;
};

}  // namespace agi
