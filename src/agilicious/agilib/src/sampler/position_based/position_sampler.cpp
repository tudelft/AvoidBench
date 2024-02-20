#include "agilib/sampler/position_based/position_sampler.hpp"

namespace agi {
PositionSampler::PositionSampler(const PositionSamplerParameters &params,
                                 const int horizon_len, const Scalar horizon_dt)
  : SamplerBase("Position Sampler", horizon_len, horizon_dt), params_(params) {}

PositionSampler::~PositionSampler() {}

bool PositionSampler::getAt(
  const QuadState &state,
  const std::vector<std::shared_ptr<ReferenceBase>> &references,
  SetpointVector *const setpoints, int horizon_len) const {
  // some sanity checks of inputs
  if (!state.valid() || setpoints == nullptr) return false;

  setpoints->clear();

  if (horizon_len < 0) {
    horizon_len = horizon_len_;
  }
  setpoints->reserve((size_t)horizon_len);

  std::vector<std::shared_ptr<ReferenceBase>>::const_iterator active_reference;

  if (!std::isfinite(prev_query_time_)) {
    active_reference = references.begin();
  } else {
    active_reference = std::lower_bound(
      references.begin(), references.end(), prev_query_time_,
      [](const std::shared_ptr<ReferenceBase> &ref, const Scalar t) -> bool {
        return ref.get()->getEndTime() <= t;
      });
  }

  Scalar t_curr = state.t;
  if (active_reference == references.end()) {
    Setpoint setpoint = references.back()->getSetpoint(state);
    for (int i = 0; i < horizon_len; ++i) {
      setpoint.state.t += t_curr;
      setpoint.input.t += t_curr;
      setpoints->push_back(setpoint);
      t_curr += horizon_dt_;
    }
    return true;
  }

  // search closest point on trajectory
  Setpoint curr_setpoint, next_setpoint;
  if (!std::isfinite(prev_query_time_)) {
    curr_setpoint = (*active_reference)->getStartSetpoint();
  } else {
    curr_setpoint = (*active_reference)->getSetpoint(prev_query_time_);
  }

  Scalar curr_dist =
    params_.axis_weights_sqrt_.cwiseProduct(curr_setpoint.state.p - state.p)
      .norm();
  Scalar next_dist = 0.0;
  Scalar t_ref_end = (*active_reference)->getEndTime();
  while (next_dist <= curr_dist + params_.search_tol_) {
    const Scalar next_time = curr_setpoint.state.t + params_.search_dt_;

    if (next_time >= t_ref_end) {
      if (++active_reference >= references.end()) {
        active_reference = std::prev(active_reference);
        break;
      }
      t_ref_end = (*active_reference)->getEndTime();
    }

    next_setpoint = (*active_reference)->getSetpoint(next_time);
    next_dist =
      params_.axis_weights_sqrt_.cwiseProduct(next_setpoint.state.p - state.p)
        .norm();
    if (next_dist <= curr_dist + params_.search_tol_) {
      curr_setpoint = next_setpoint;
    }
  }

  // closest point is found, now fill the horizon as for the time sampler
  prev_query_time_ = curr_setpoint.state.t;
  t_curr = curr_setpoint.state.t;

  t_ref_end = (*active_reference)->getEndTime();
  for (int i = 0; i < horizon_len; i++) {
    Setpoint setpoint;
    if (t_curr >= t_ref_end) {
      if (++active_reference >= references.end())
        active_reference = std::prev(active_reference);
      t_ref_end = (*active_reference)->getEndTime();
    }

    setpoints->emplace_back(
      (*active_reference)->getSetpoint(curr_setpoint.state, t_curr));
    t_curr += horizon_dt_;
  }

  // update the setpoint timestamps
  for (auto &setpoint : *setpoints) {
    setpoint.state.t += state.t - prev_query_time_;
    setpoint.input.t += state.t - prev_query_time_;
  }

  return true;
}


}  // namespace agi
