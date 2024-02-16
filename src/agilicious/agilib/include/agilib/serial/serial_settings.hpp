#pragma once

#include <iostream>
#include <string>

namespace agi {

enum class SerialMode { Read, Write, ReadWrite };

class SerialSettings {
 public:
  explicit SerialSettings(const std::string& port, const int baudrate,
                          const SerialMode serial_mode = SerialMode::Write)
    : baudrate(baudrate), serial_mode(serial_mode), port(port) {}

  explicit SerialSettings(const std::string& port, const int baudrate,
                          const char start_delimiter, const char end_delimiter,
                          const SerialMode serial_mode = SerialMode::ReadWrite)
    : baudrate(baudrate),
      serial_mode(serial_mode),
      start_delimiter(start_delimiter),
      end_delimiter(end_delimiter),
      use_delimiter(true),
      check_multiple_delimiters(true),
      port(port) {}

  friend std::ostream& operator<<(std::ostream& os,
                                  const SerialSettings& settings) {
    os << "SerialSettings:\n"
       << "Port:     " << settings.port << "\n"
       << "Baudrate: " << settings.baudrate << "\n"
       << "Mode      " << (int)settings.serial_mode << std::endl;

    if (settings.use_delimiter) {
      os << "Start Delimiter: " << settings.start_delimiter << "\n"
         << "End Delimiter:  " << settings.end_delimiter << "\n"
         << "Check multiple delimiters: "
         << (settings.check_multiple_delimiters ? "yes" : "no") << std::endl;
    } else {
      os << "Not using delimiters!" << std::endl;
    }

    return os;
  }

  int baudrate;
  SerialMode serial_mode{SerialMode::Write};
  bool sbus{false};
  char start_delimiter{0x00};
  char end_delimiter{0x00};
  bool use_delimiter{false};
  bool check_multiple_delimiters{false};
  std::string port;

 protected:
  SerialSettings(const std::string& port)  // constructor for SBUS child class.
    : baudrate(100000),
      serial_mode(SerialMode::Write),
      sbus(true),
      use_delimiter(false),
      port(port) {}
};

class SbusSerialSettings : public SerialSettings {
 public:
  SbusSerialSettings(const std::string& port) : SerialSettings(port) {}
};

}  // namespace agi
