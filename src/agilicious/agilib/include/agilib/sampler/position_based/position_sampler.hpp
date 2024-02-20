#pragma once

#include "agilib/sampler/position_based/position_sampler_params.hpp"
#include "agilib/sampler/sampler_base.hpp"

namespace agi {

class PositionSampler : public SamplerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionSampler(const PositionSamplerParameters &params,
                  const int horizon_len, const Scalar horizon_dt);
  PositionSampler(const PositionSamplerParameters &params)
    : PositionSampler(params, 10, 0.1){};
  ~PositionSampler();

  bool getAt(const QuadState &state,
             const std::vector<std::shared_ptr<ReferenceBase>> &references,
             SetpointVector *const setpoints, int horizon_len = -1) const;

  bool isTimeBased() const { return false; }

 private:
  mutable Scalar prev_query_time_ = NAN;
  PositionSamplerParameters params_;
};

}  // namespace agi