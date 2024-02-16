#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "agilib/bridge/laird/laird_bridge_params.hpp"
#include "agilib/bridge/laird/laird_encoding.hpp"
#include "agilib/bridge/serial_bridge.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {
class LairdBridge : public SerialBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LairdBridge(const LairdParams& params, const TimeFunction time_function);

  bool receiveCommand(Command* const command, bool* const armed);
  bool waitForData();

 protected:
  using SerialBridge::sendCommand;
  virtual bool sendCommand(const Command& command, const bool active) override;
  void receiveCallback(const char* const buffer, const int length);

  LairdEncoding encoding_;

  std::mutex wait_for_data_mtx_;
  std::condition_variable wait_for_data_cv_;

  static constexpr int BUFFER_SIZE = SerialBridge::BUFFER_SIZE;
  static constexpr int MSG_SIZE = LairdEncoding::BUFFER_SIZE;

  std::mutex buffer_mtx_;
  char buffer_[BUFFER_SIZE];
  int buffer_length_{0};
};

}  // namespace  agi
