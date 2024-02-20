#pragma once

#include <iostream>

// Agilib
#include "agilib/math/math.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

template<int rows>
struct BrentFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~BrentFunction() = default;
  virtual ArrayVector<rows> evaluate(
    const Ref<const ArrayVector<rows>>) const = 0;

  static constexpr int SIZE = rows;
};


template<typename Function>
class Brent {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setTolerance(const Scalar tol);
  int find_root(const Function&, const Scalar range_min, const Scalar range_max,
                const ArrayVector<Function::SIZE>& initial_guess,
                const bool warmstart,
                Ref<ArrayVector<Function::SIZE>>&& result) const;


  static constexpr int SUCCESS = 1;
  static constexpr int CONTINUE = 0;
  static constexpr int ERROR = 2;

 private:
  int step(const int i, ArrayVector<Function::SIZE>& f_x0,
           ArrayVector<Function::SIZE>& f_x1, ArrayVector<Function::SIZE>& f_x2,
           ArrayVector<Function::SIZE>& x0, ArrayVector<Function::SIZE>& x1,
           ArrayVector<Function::SIZE>& x2, ArrayVector<Function::SIZE>& x3,
           ArrayVector<Function::SIZE>& x4) const;

  Logger logger_{"Brent"};

  static constexpr int max_iter_ = 100;
  Scalar tol_ = 1E-3;
  Scalar warmstart_delta_ = 0.02;
};


template<typename Function>
void Brent<Function>::setTolerance(const Scalar tol) {
  this->tol_ = tol;
}

template<typename Function>
int Brent<Function>::find_root(
  const Function& fcn, const Scalar range_min, const Scalar range_max,
  const ArrayVector<Function::SIZE>& initial_guess, const bool warmstart,
  Ref<ArrayVector<Function::SIZE>>&& result) const {
  // Start up
  bool err_flag = false;
  Scalar scale = range_max - range_min;
  ArrayVector<Function::SIZE> x0, x1, x2, x3, x4;
  ArrayVector<Function::SIZE> f_x0, f_x1, f_x2;
  if (warmstart) {
    x0 = initial_guess - scale * warmstart_delta_;
    x1 = initial_guess + scale * warmstart_delta_;
    f_x0 = fcn.evaluate(x0);
    f_x1 = fcn.evaluate(x1);
  }
  if (!warmstart || (f_x0 * f_x1 > 0).any()) {
    if (warmstart) logger_.debug("Warmstarting failed!");
    x0.setConstant(range_min);
    x1.setConstant(range_max);
    f_x0 = fcn.evaluate(x0);
    f_x1 = fcn.evaluate(x1);
  }


  if ((f_x0 * f_x1 > 0).any()) {
    err_flag = true;
  }

  x2 = x0;
  f_x2 = f_x0;

  x3 = x1 - x0;
  x4 = x3;

  // Solving
  int iter = 0;
  ArrayVector<Function::SIZE> retVal;
  retVal.setZero();
  while (!(retVal == 1).all() && !(retVal == 2).any() && iter < max_iter_ &&
         !err_flag) {
    for (int i = 0; i < Function::SIZE; ++i) {
      retVal(i) = step(i, f_x0, f_x1, f_x2, x0, x1, x2, x3, x4);
    }
    f_x1 = fcn.evaluate(x1);
    ++iter;
  }
  result = x1;
  return retVal.maxCoeff();
}

template<typename Function>
int Brent<Function>::step(const int i, ArrayVector<Function::SIZE>& f_x0,
                          ArrayVector<Function::SIZE>& f_x1,
                          ArrayVector<Function::SIZE>& f_x2,
                          ArrayVector<Function::SIZE>& x0,
                          ArrayVector<Function::SIZE>& x1,
                          ArrayVector<Function::SIZE>& x2,
                          ArrayVector<Function::SIZE>& x3,
                          ArrayVector<Function::SIZE>& x4) const {
  if (i > Function::SIZE) {
    return ERROR;
  }


  if (f_x1(i) * f_x2(i) > 0) {
    x2(i) = x0(i);
    f_x2(i) = f_x0(i);
    x3(i) = x1(i) - x0(i);
    x4(i) = x3(i);
  }

  if (std::abs(f_x2(i)) < std::abs(f_x1(i))) {
    x0(i) = x1(i);
    x1(i) = x2(i);
    x2(i) = x0(i);

    f_x0(i) = f_x1(i);
    f_x1(i) = f_x2(i);
    f_x2(i) = f_x0(i);
  }

  const Scalar m = 0.5 * (x2(i) - x1(i));

  if (std::abs(m) > tol_ && std::abs(f_x1(i)) > 0) {
    if (std::abs(x4(i)) < tol_ || std::abs(f_x0(i)) <= std::abs(f_x1(i))) {
      x3(i) = m;
      x4(i) = m;
    } else {
      Scalar s = f_x1(i) / f_x0(i);
      Scalar p, q, r;
      if (x0(i) == x2(i)) {
        p = 2 * m * s;
        q = 1 - s;
      } else {
        q = f_x0(i) / f_x2(i);
        r = f_x1(i) / f_x2(i);
        p = s * (2 * m * q * (q - r) - (x1(i) - x0(i)) * (r - 1));
        q = (q - 1) * (r - 1) * (s - 1);
      }
      if (p > 0) {
        q = -q;
      } else {
        p = -p;
      }
      s = x4(i);
      x4(i) = x3(i);
      if ((2 * p < 3 * m * q - std::abs(tol_ * q)) &&
          (p < std::abs(s * q / 2))) {
        x3(i) = p / q;
      } else {
        x3(i) = m;
        x4(i) = m;
      }
    }

    x0(i) = x1(i);
    f_x0(i) = f_x1(i);

    if (std::abs(x3(i)) > tol_) {
      x1(i) += x3(i);
    } else {
      if (m > 0) {
        x1(i) += tol_;
      } else {
        x1(i) -= tol_;
      }
    }
  } else {
    return SUCCESS;
  }
  return CONTINUE;
}

}  // namespace agi
