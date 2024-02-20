#pragma once

#include <memory>

#include "agilib/bridge/thrust_map.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {
class MsgEncoding {
 public:
  virtual ~MsgEncoding() = default;
  virtual bool encode(const Command& command, const bool armed,
                      char* const buffer, int* const length) const = 0;

  virtual bool decode(const char* const buffer, const int length,
                      Command* const command, bool* const armed) const {
    return false;
  }
};

}  // namespace agi
