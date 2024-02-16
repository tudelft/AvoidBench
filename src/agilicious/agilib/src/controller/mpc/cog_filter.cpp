#include "agilib/controller/mpc/cog_filter.hpp"

namespace agi {

CogFilter::CogFilter(const Quadrotor& quad, const Matrix<NX, NX>& Q_init,
                     const Matrix<NX, NX>& Q, const Matrix<NOMEGA, NOMEGA>& R)
  : quad_(quad), Q_init_(Q_init), Q_(Q), R_(R) {
  reset();
}

void CogFilter::reset() {
  imus_.clear();
  commands_.clear();

  x_.setZero();
  P_ = Q_init_;
}

bool CogFilter::addImu(const ImuSample& sample) {
  if (!sample.valid()) {
    logger_.warn("Got invalid IMU sample!");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  imus_.push_back(sample);
  if ((int)imus_.size() > N_max_samples_) imus_.pop_front();

  return true;
}
bool CogFilter::addCommand(const Command& command) {
  if (!command.isSingleRotorThrusts()) {
    logger_.warn("Got invalid command!");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  commands_.push_back(command);
  if ((int)commands_.size() > N_max_samples_) commands_.pop_front();
  return true;
}

bool CogFilter::update() {
  // Necessary for prediction

  if (imus_.empty() || commands_.empty()) return true;

  std::lock_guard<std::mutex> lock(mutex_);
  // logger_.info("Updating: IMU queue: %zu,   Command queue: %zu",
  // imus_.size(),
  //              commands_.size());

  std::deque<ImuSample>::iterator imu_it = imus_.begin();
  std::deque<Command>::iterator command_it = commands_.begin();

  // Purge IMU sample swithout command.
  while (imu_it->t < command_it->t && imu_it < imus_.end()) {
    // logger_.info("Cleared an IMU sample at %1.6f", imu_it->t);
    imu_it++;
  }
  if (imu_it >= imus_.end()) return true;

  // Pull state forward to oldest IMU sample.
  // If the state was recently updated, t_ = imut_it->t and the if is jumped.
  // This is because, as above, the first IMU sample should be the last
  // processed one.
  if (!std::isfinite(t_) || t_ < imu_it->t) {
    x_.head(NOMEGA) = imu_it->omega.head(NOMEGA);
    t_ = imu_it->t;
    // logger_.info("Pulled state to %1.3f", t_);
  }

  // Forward imu_it to the first sample that has to be processed.
  std::deque<ImuSample>::iterator previous_imu = imu_it;
  while (imu_it->t <= t_ && imu_it < imus_.end()) {
    previous_imu = imu_it;
    imu_it++;
  }
  if (imu_it >= imus_.end()) return true;

  // Iterate over all IMU samples that have to be processed.
  bool success = true;
  std::deque<Command>::iterator previous_command = command_it;
  const Scalar t_most_recent_command = commands_.back().t;

  while (imu_it < imus_.end() && command_it < commands_.end()) {
    // If the IMU is newer than the latest command, keep the last and exit.
    const Scalar t_target = imu_it->t;
    if (t_target > t_most_recent_command) break;


    // Advance to the first relevant command sample
    while (command_it->t < t_ && command_it < commands_.end()) {
      previous_command = command_it;
      command_it++;
    }

    // Predict forward to t_target;
    while (t_ < t_target && command_it < commands_.end()) {
      const Scalar t_new = std::min(command_it->t, t_target);
      const Scalar dt = std::max(t_new - t_, 0.0);

      Matrix<NX, NX> F = Matrix<NX, NX>::Zero();
      F(0, 3) = previous_command->thrusts.sum() / quad_.J_(0, 0);
      F(1, 2) = -previous_command->thrusts.sum() / quad_.J_(1, 1);

      x_ += dt * F * x_ + dt * B_ * previous_command->thrusts;
      const Matrix<NX, NX> IdtF = Matrix<NX, NX>::Identity() + dt * F;
      P_ = IdtF * P_ * IdtF.transpose() + dt * dt * Q_;
      P_ = 0.5 * (P_ + P_.transpose());
      // logger_.info("Predicted from %1.3f to %1.3f over %1.3f", t_, t_new,
      // dt); logger_.info("with target %1.3f and command %1.3f", t_target,
      //              command_it->t);
      // logger_ << "P prior" << std::endl << P_ << std::endl;
      t_ = t_new;
      if (t_ < t_target) {
        previous_command = command_it;
        command_it++;
      }
    }

    // Check if we predicted sufficiently far.
    if (t_ != t_target) {
      // logger_.info("Did not reach target %1.3f of %1.3f", t_target, t_);
      break;
    }

    // Perform posterior update
    const Vector<NY> z = imu_it->omega.head(NOMEGA) - H_ * x_;
    const Matrix<NY, NY> S = H_ * P_ * H_.transpose() + R_;
    bool invertible = false;
    Matrix<NY, NY> S_inv;
    S.computeInverseWithCheck(S_inv, invertible);
    if (!invertible) {
      logger_.warn("Could not compute matrix inverse!");
      success = false;
      break;
    }

    const Matrix<NX, NY> K = P_ * H_.transpose() * S.inverse();
    // logger_ << "H" << std::endl << H_ << std::endl;
    // logger_ << "P" << std::endl << P_ << std::endl;
    // logger_ << "S" << std::endl << S << std::endl;
    // logger_ << "K" << std::endl << K << std::endl;
    // logger_ << "z" << std::endl << z << std::endl;
    x_ += K * z;
    P_ -= K * H_ * P_;
    P_ = 0.5 * (P_ + P_.transpose());

    // logger_.info("Updated at %1.3f", t_);
    // logger_ << x_.transpose() << std::endl;

    // Ensure all commands are used until their time.
    command_it = previous_command;

    // Advance IMU iterator.
    previous_imu = imu_it;
    imu_it++;
  }

  // Clear the queue up to the previous samples.
  if (!imus_.empty()) imus_.erase(imus_.begin(), previous_imu);
  if (!commands_.empty()) commands_.erase(commands_.begin(), previous_command);

  return success;
}

Vector<2> CogFilter::getLengthCorrection() const { return x_.tail(NL); }

Matrix<3, 4> CogFilter::getAbsolutLengths() const {
  const Vector<3> dl(x_(2), x_(3), 0);
  return quad_.t_BM_ + dl.replicate(1, 4);
}

}  // namespace agi
