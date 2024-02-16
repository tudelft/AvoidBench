#pragma once

#include <deque>
#include <mutex>

#include "agilib/math/types.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

class CogFilter {
 private:
  static constexpr int NX = 4;
  static constexpr int NU = 4;
  static constexpr int NL = 2;
  static constexpr int NY = 2;
  static constexpr int NOMEGA = 2;

 public:
  CogFilter(const Quadrotor& quad, const Matrix<NX, NX>& Q_init,
            const Matrix<NX, NX>& Q, const Matrix<NOMEGA, NOMEGA>& R);

  void reset();
  bool addImu(const ImuSample& sample);
  bool addCommand(const Command& command);

  bool update();

  Vector<NL> getLengthCorrection() const;
  Matrix<3, 4> getAbsolutLengths() const;

 private:
  Logger logger_{"CoG Filter"};
  const Quadrotor quad_;
  const Matrix<NX, NX> Q_init_;
  const Matrix<NX, NX> Q_;
  const Matrix<NY, NY> R_;

  static constexpr int N_max_samples_ = 128;

  std::mutex mutex_;
  std::deque<ImuSample> imus_;
  std::deque<Command> commands_;

  Scalar t_{NAN};
  Vector<NX> x_ = Vector<NX>::Zero();
  Matrix<NX, NX> P_ = Matrix<NX, NX>::Zero();

  const Matrix<NY, NX> H_ =
    (Matrix<NY, NX>() << Matrix<NY, NOMEGA>::Identity(), Matrix<NY, NL>::Zero())
      .finished();
  const Matrix<NX, NU> B_ =
    (Matrix<NX, NU>() << 1.0 / quad_.J_(0, 0) * quad_.t_BM_.row(1),
     -1.0 / quad_.J_(1, 1) * quad_.t_BM_.row(0), Matrix<NL, NU>::Zero())
      .finished();
};

}  // namespace agi
