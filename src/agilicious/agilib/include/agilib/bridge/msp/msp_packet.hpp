#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "agilib/bridge/msp/msp_msgs_defs.hpp"

namespace agi {

struct MspPacket {
  MspPacket(const char payload_bytes);
  ~MspPacket();

  void finalize();
  bool copy_to_buffer(char* const buffer) const;
  friend std::ostream& operator<<(std::ostream& os, const MspPacket& p);

  // Read access
  unsigned char direction() const { return data[2]; }

  unsigned char payload_length() const { return data[3]; }

  unsigned char message_size() const { return data[3] + 6; }

  // MSP Message code as defined in msp_msgs_degs.hpp
  unsigned char& code() const { return data[4]; }

  template<typename T = unsigned char>
  const T* const payload() const {
    return reinterpret_cast<const T* const>(&data[5]);
  }

  unsigned char& checksum() const { return data[data[3] + 5]; }

  const void* message() const { return reinterpret_cast<const void*>(data); }

  // Write access through references and pointers
  unsigned char& direction() { return data[2]; }

  unsigned char& code() { return data[4]; }

  template<typename T = unsigned char>
  T* payload() {
    return reinterpret_cast<T* const>(&data[5]);
  }

  unsigned char& checksum() { return data[data[3] + 5]; }

 private:
  unsigned char* data;
};

}  // namespace agi
