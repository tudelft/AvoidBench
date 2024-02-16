#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "agilib/serial/serial_settings.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

class SerialPort {
 public:
  explicit SerialPort(const SerialSettings& settings);
  ~SerialPort();

  bool start();
  bool stop();
  bool isOpen() const;

  int send(const char* const buffer, const int length);
  int receive(char* const buffer);
  bool addReceiveCallback(
    std::function<void(const char* const msg, const int length)> function);
  static constexpr int BUFFER_SIZE = 1024;

 private:
  bool configure(const int baudrate);
  void serialThread();

  const SerialSettings settings_;
  std::function<void(const char* const msg, const int length)>
    receive_callback_;

  int serial_fd_{-1};
  bool should_exit_{false};

  std::thread serial_thread_;

  static constexpr size_t BS = BUFFER_SIZE;

  char receive_buffer_[BS];
  size_t received_length_{0u};

  char received_message_[BS];
  size_t received_message_length_{0u};

  std::mutex rc_mtx_;
  std::mutex proc_mtx_;
  std::mutex buffer_mtx;
  std::condition_variable rc_cv_;
  std::condition_variable proc_cv_;

  Logger logger_{"SerialPort"};
};

}  // namespace agi