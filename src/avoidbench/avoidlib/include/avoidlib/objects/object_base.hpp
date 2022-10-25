#pragma once

#include "avoidlib/common/types.hpp"

namespace avoidlib {

class ObjectBase {
 public:
  ObjectBase();
  virtual ~ObjectBase();

  // reset
  virtual bool reset(void) = 0;

  // run
  virtual bool run(const Scalar dt) = 0;
};

}  // namespace avoidlib
