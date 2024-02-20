#pragma once

#include "agilib/bridge/ctrl/ctrl_bridge_params.hpp"
#include "agilib/bridge/ctrl/ctrl_encoding.hpp"
#include "agilib/bridge/serial_bridge.hpp"
#include "agilib/types/feedback.hpp"

namespace agi {

class CtrlBridge : public SerialBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CtrlBridge(const Quadrotor& quad, const CtrlParams& params,
             const TimeFunction time_function);

  virtual bool getFeedback(Feedback* const feedback = nullptr) override;

 protected:
  using SerialBridge::sendCommand;
  virtual bool sendCommand(const Command& command, const bool active) override;
  void receiveCallback(const char* const buffer, const int length);
  void decodeBuffer();

  CtrlEncoding encoding_;
};

}  // namespace agi
