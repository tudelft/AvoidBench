#pragma once

#include <cmath>
#include <functional>
#include <memory>

// Agilib
#include "agilib/math/fast_atan2.hpp"
#include "agilib/utils/timer.hpp"

// BEM
#include "agilib/simulator/bem/brent.hpp"
#include "agilib/simulator/bem/gauss_kronrod.hpp"
#include "agilib/simulator/bem/propeller_state.hpp"

namespace agi {


struct IntegrandParam {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArrayVector<QS::NMOT> vind;
  Scalar radius;
};

class IntegrandPsi : public GaussKronrodFunction<QS::NMOT> {
 public:
  enum TYPE : int { THRUST = 1, TORQUE = 2, HFORCE = 3 };

  IntegrandPsi(const TYPE type) : type_{type} {}

  void setState(std::shared_ptr<PropellerState>& pstate) { pstate_ = pstate; }

  Array<QS::NMOT, 15> evaluate(const Ref<const ArrayVector<15>> Psi,
                               const void* const param) const override;

 private:
  std::shared_ptr<PropellerState> pstate_;
  const TYPE type_;
};

class IntegrandR : public GaussKronrodFunction<QS::NMOT> {
 public:
  IntegrandR() = default;
  void setState(std::shared_ptr<PropellerState>& pstate,
                std::shared_ptr<IntegrandPsi>& fPsi) {
    pstate_ = pstate;
    fPsi_ = fPsi;
  }

  Array<4, 15> evaluate(const Ref<const ArrayVector<15>> r,
                        const void* const param = nullptr) const override;


 private:
  std::shared_ptr<PropellerState> pstate_;
  std::shared_ptr<GaussKronrodFunction<QS::NMOT>> fPsi_;
  const GaussKronrod<GaussKronrodFunction<QS::NMOT>> gk_{};
};


class ThrustFunction : public BrentFunction<QS::NMOT> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThrustFunction() = default;
  void setState(std::shared_ptr<PropellerState>& pstate,
                std::shared_ptr<IntegrandR>& fR) {
    pstate_ = pstate;
    fR_ = fR;
  }

  ArrayVector<4> evaluate(const Ref<const ArrayVector<4>> vind) const override;


 private:
  std::shared_ptr<PropellerState> pstate_;
  std::shared_ptr<GaussKronrodFunction<QS::NMOT>> fR_;
  const GaussKronrod<GaussKronrodFunction<QS::NMOT>> gk_{};
};

}  // namespace agi
