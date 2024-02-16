#include "agilib/serial/serial.hpp"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace agi {

SerialPort::SerialPort(const SerialSettings& settings) : settings_(settings) {
  start();
}

SerialPort::~SerialPort() { stop(); }

bool SerialPort::start() {
  serial_fd_ = open(settings_.port.c_str(), O_RDWR | O_NOCTTY);
  usleep(10000);

  if (serial_fd_ == -1) {
    logger_.error("Could not open port %s!", settings_.port.c_str());
    return false;
  }

  if (!configure(settings_.baudrate)) {
    logger_.error("[%s] Could not configure serial port %s with baudrate %d",
                  settings_.port.c_str(), settings_.baudrate);
    return false;
  }

  // Clear buffer brute force way.
  char trash_buffer[BS];
  while (read(serial_fd_, trash_buffer, BUFFER_SIZE) > 0) {
  }

  should_exit_ = false;

  if (settings_.serial_mode != SerialMode::Write)
    serial_thread_ = std::thread(&SerialPort::serialThread, this);

  return true;
}

bool SerialPort::stop() {
  if (serial_thread_.joinable()) {
    should_exit_ = true;
    serial_thread_.join();
  }
  should_exit_ = true;

  if (serial_fd_) close(serial_fd_);

  return true;
}

bool SerialPort::isOpen() const {
  return fcntl(serial_fd_, F_GETFD) != -1 || errno != EBADF;
}

bool SerialPort::configure(const int baudrate) {
  // clear config
  fcntl(serial_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_fd_, F_SETFL, FNDELAY);

  struct termios2 uart_config;

  // Get config
  ioctl(serial_fd_, TCGETS2, &uart_config);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  uart_config.c_iflag &=
    ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // For custom baudrate and SBUS
  // clear current char size mask, no parity checking,
  uart_config.c_cflag &= ~(CSIZE | PARENB);
  // No output processing, force 8 bit input
  uart_config.c_cflag |= CS8;
  // Set two stop bits, rather than one.
  uart_config.c_cflag |= CSTOPB;
  if (settings_.sbus) {
    // Enable parity generation on output and parity checking for input.
    uart_config.c_cflag |= PARENB;
  }

  // Enable a non standard baud rate
  uart_config.c_cflag &= ~CBAUD;
  uart_config.c_cflag |= BOTHER;
  uart_config.c_ispeed = baudrate;
  uart_config.c_ospeed = baudrate;

  // Finally, apply the configuration
  if (ioctl(serial_fd_, TCSETS2, &uart_config) < 0) {
    logger_.error("Could not set configuration of serial port!");
    return false;
  }
  return true;
}

void SerialPort::serialThread() {
  while (!should_exit_) {
    size_t last_length = received_length_;
    const ssize_t bytes_read = read(
      serial_fd_, &receive_buffer_[received_length_], BS - received_length_);

    if (bytes_read > 0) received_length_ += bytes_read;
    if (received_length_ <= (size_t)(settings_.use_delimiter)) continue;

    if (received_message_length_ >= BS - received_length_)
      received_message_length_ = 0;

    if (settings_.use_delimiter) {
      // If we don't have a container start yet, move to the first delimiter.
      for (size_t idx = 0; idx < received_length_; ++idx) {
        // If we have a delimiter...
        if (receive_buffer_[idx] == settings_.start_delimiter) {
          if (settings_.check_multiple_delimiters) {
            // ... and catch the double delimiter case
            while ((idx + 1) < received_length_ &&
                   receive_buffer_[idx + 1] == settings_.start_delimiter) {
              ++idx;
            }
          }
          char tmp[BS];
          received_length_ -= idx;
          last_length = 0u;
          memcpy(tmp, &receive_buffer_[idx], received_length_);
          memcpy(receive_buffer_, tmp, received_length_);
          break;
        }
      }

      // If there is nothing to do, poll again.
      if (received_length_ == 0u) continue;

      // If we haven't found a start, trash all received data
      if (receive_buffer_[0] != settings_.start_delimiter) {
        received_length_ = 0u;
        continue;
      }

      // Now we have a delimiter at the beginning and can search for the end
      for (size_t idx = std::max((size_t)1, last_length);
           idx < received_length_; ++idx) {
        if (receive_buffer_[idx] == settings_.end_delimiter) {
          // End found, copy into output buffer and notify possible receiver.
          const std::lock_guard<std::mutex> guard(buffer_mtx);
          memcpy(received_message_, &receive_buffer_[1], idx - 1u);
          received_message_length_ = idx - 1u;
          rc_cv_.notify_all();

          char tmp[BS];
          received_length_ -= idx;
          last_length = 0u;
          memcpy(tmp, &receive_buffer_[idx], received_length_);
          memcpy(receive_buffer_, tmp, received_length_);
          break;
        }
      }
    } else {
      memcpy(&received_message_[received_message_length_], &receive_buffer_,
             received_length_);
      received_message_length_ += received_length_;
      received_length_ = 0u;
    }

    if (received_message_length_ > 0u && receive_callback_) {
      receive_callback_(received_message_, received_message_length_);
      received_message_length_ = 0u;
      continue;
    }

    // If we have a start but no end in a full buffer, something went wrong.
    // We trash the data and wait for new data.
    if (received_length_ == BS) {
      received_length_ = 0;
      continue;
    }
    std::unique_lock<std::mutex> lck(proc_mtx_);
    proc_cv_.wait_for(lck, std::chrono::microseconds(50));
  }
}

int SerialPort::send(const char* const buffer, const int length) {
  const ssize_t bytes_written = write(serial_fd_, buffer, (size_t)length);
  if (bytes_written < 0) {
    logger_.error("Could not write to serial port!");
  } else if ((int)bytes_written < length) {
    logger_.error("Could only write %d of %d bytes!", (int)bytes_written,
                  length);
  }

  return (int)bytes_written;
}

int SerialPort::receive(char* const buffer) {
  if (settings_.serial_mode == SerialMode::Write) return -1;

  if (!received_message_length_) {
    std::unique_lock<std::mutex> lck(rc_mtx_);
    rc_cv_.wait_for(lck, std::chrono::milliseconds(2));
  }

  const size_t return_length = received_message_length_;
  if (received_message_length_) {
    const std::lock_guard<std::mutex> guard(buffer_mtx);
    memcpy(buffer, received_message_, received_message_length_);
    received_message_length_ = 0u;
  }

  proc_cv_.notify_all();
  return (int)return_length;
}

bool SerialPort::addReceiveCallback(
  std::function<void(const char* const msg, const int length)> function) {
  if (settings_.serial_mode == SerialMode::Write) return false;
  receive_callback_ = function;
  return true;
}

}  // namespace agi
