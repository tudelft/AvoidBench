#pragma once

#include <condition_variable>
#include <mutex>

#include "agilib/controller/controller_base.hpp"
#include "agilib/controller/mpc/cog_filter.hpp"
#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/controller/mpc/wrapper.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

class MpcController : public ControllerBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  MpcController(const Quadrotor& quad,
                const std::shared_ptr<MpcParameters>& params,
                const Scalar exec_dt = 0.01);

  bool getCommand(const QuadState& state, const SetpointVector& reference,
                  SetpointVector* const setpoints) override;
  bool reset();
  bool reset(const QuadState& state);

  virtual void addImuSample(const ImuSample& sample) override;

  bool updateParameters(const Quadrotor& quad,
                        const std::shared_ptr<MpcParameters>& params);
  bool updateParameters(const Quadrotor& params);
  bool updateParameters(const std::shared_ptr<MpcParameters>& params);
  std::shared_ptr<MpcParameters> getParameters();

  void printTiming() const;
  void logTiming() const override;

 private:
  using Wrapper = acados::MpcWrapper;

  bool reset(const Vector<Wrapper::NX>& state);
  Scalar lerp(const Scalar a, const Scalar b, const Scalar t) const;

  Quadrotor quad_;
  std::shared_ptr<MpcParameters> params_;
  Wrapper wrapper_;
  CogFilter cog_filter_;

  // Thread-safety and preparation.
  std::mutex acados_mutex_;

  Timer timing_solver_{"Solver"};
  Timer timing_update_{"Update"};

  const Vector<Wrapper::NX> hover_state_ =
    (Vector<Wrapper::NX>() << Vector<3>::Zero(), Vector<4>(1, 0, 0, 0),
     Vector<3>::Zero(), Vector<3>::Zero())
      .finished();
  const Vector<4> hover_input_acc_ = Vector<4>::Constant(G / 4.0);
  Vector<4> hover_input_{quad_.m_ * hover_input_acc_};
};

}  // namespace agi
