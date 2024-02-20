#pragma once

#include <agilib/math/types.hpp>
#include <cmath>

namespace agi {

// Fast signed atan approximation from
// https://www.dsprelated.com/showarticle/1052.php
template<typename T>
struct ApproxAtan2 {
  EIGEN_EMPTY_STRUCT_CTOR(ApproxAtan2)
  T operator()(const T yf, const T xf) const {
    const float x = (float)xf;
    const float y = (float)yf;
    static constexpr float n1 = 0.97239411f;
    static constexpr float n2 = -0.19194795f;
    float result = 0.0f;
    if (x != 0.0f) {
      const union {
        float flVal;
        uint nVal;
      } tYSign = {y};
      const union {
        float flVal;
        uint nVal;
      } tXSign = {x};
      if (fabsf(x) >= fabsf(y)) {
        union {
          float flVal;
          uint nVal;
        } tOffset = {M_PI};
        // Add or subtract PI based on y's sign.
        tOffset.nVal |= tYSign.nVal & 0x80000000u;
        // No offset if x is positive, so multiply by 0 or based on x's sign.
        tOffset.nVal *= tXSign.nVal >> 31;
        result = tOffset.flVal;
        const float z = y / x;
        result += (n1 + n2 * z * z) * z;
      } else  // Use atan(y/x) = pi/2 - atan(x/y) if |y/x| > 1.
      {
        union {
          float flVal;
          uint nVal;
        } tOffset = {M_PI_2};
        // Add or subtract PI/2 based on y's sign.
        tOffset.nVal |= tYSign.nVal & 0x80000000u;
        result = tOffset.flVal;
        const float z = x / y;
        result -= (n1 + n2 * z * z) * z;
      }
    } else if (y > 0.0f) {
      result = M_PI_2;
    } else if (y < 0.0f) {
      result = -M_PI_2;
    }
    return result;
  }
};

template<typename T>
T fastApproxAtan2(const T x, const T y) {
  return ApproxAtan2<T>::operator()(x, y);
}


}  // namespace agi
