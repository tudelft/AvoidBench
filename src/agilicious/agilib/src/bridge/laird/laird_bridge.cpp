#include "agilib/bridge/laird/laird_bridge.hpp"

#include <unistd.h>

#include <chrono>

namespace agi {

LairdBridge::LairdBridge(const LairdParams& params,
                         const TimeFunction time_function)
  : SerialBridge(params.serial_settings, "Laird Bridge", time_function,
                 params.timeout, 0),
    encoding_(params.single_rotor_thrust) {
  if (!isOpen()) {
    logger_.error("Could not open serial port");
    logger_ << params.serial_settings;
  } else {
    if (serial_settings_.serial_mode != SerialMode::Write) {
      logger_.info("Start Laird in receive mode!");
      if (!serial_port_.addReceiveCallback(
            std::bind(&LairdBridge::receiveCallback, this,
                      std::placeholders::_1, std::placeholders::_2))) {
        logger_.error("Could not bind receive callback to serial port!");
      }
    } else {
      logger_.info("Start Laird in transmit mode!");
    }
  }
}

bool LairdBridge::sendCommand(const Command& command, const bool active) {
  return sendCommand(command, active, encoding_);
}

bool LairdBridge::receiveCommand(Command* const command, bool* const armed) {
  if (command == nullptr || armed == nullptr) return false;

  std::lock_guard<std::mutex> lock(buffer_mtx_);

  if (buffer_length_ < MSG_SIZE) return false;

  int bytes_read;
  bool got_command = false;

  // Iterate over the buffer from the start until we find a valid message.
  for (bytes_read = 0; bytes_read <= buffer_length_ - MSG_SIZE; ++bytes_read) {
    // Try to decode the message, if successful, add the read message length.
    if (encoding_.decode(&buffer_[bytes_read], buffer_length_ - bytes_read,
                         command, armed)) {
      got_command = true;
      break;
    }
  }

  if (bytes_read > 0) logger_.info("Dropped %d bytes!", bytes_read);
  bytes_read += got_command ? MSG_SIZE : 0;

  // Delete what was parsed from the buffer and make sure length is valid.
  static char tmp[BUFFER_SIZE];
  buffer_length_ -= bytes_read;
  if (buffer_length_ > 0) {
    memcpy(&tmp, &buffer_[bytes_read], buffer_length_);
    memcpy(buffer_, &tmp, buffer_length_);
  } else {
    buffer_length_ = 0;
  }

  return got_command;
}

bool LairdBridge::waitForData() {
  if (buffer_length_ >= MSG_SIZE) return true;
  std::unique_lock<std::mutex> lock(wait_for_data_mtx_);
  return wait_for_data_cv_.wait_for(lock, std::chrono::milliseconds(100),
                                    [&] { return buffer_length_ >= MSG_SIZE; });
}

void LairdBridge::receiveCallback(const char* const buffer, const int length) {
  std::lock_guard<std::mutex> lock(buffer_mtx_);
  if (length > BUFFER_SIZE - buffer_length_) {
    buffer_length_ = 0;
  }
  if (length > BUFFER_SIZE) return;

  memcpy(&buffer_[buffer_length_], buffer, length);
  buffer_length_ += length;
  wait_for_data_cv_.notify_all();
}

}  // namespace agi
