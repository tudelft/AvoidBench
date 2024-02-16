#pragma once

#include <Eigen/Dense>
#include "agilib/types/setpoint.hpp"
#include "agilib/reference/reference_base.hpp"

namespace agi {
struct TrajectoryExtPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;
  Eigen::Quaterniond attitude;
  Eigen::Vector3d bodyrates;
  double collective_thrust;
  double time_from_start;
};

enum class FrameID { Body, World };


class TrajectoryExt : public ReferenceBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  TrajectoryExt();
  TrajectoryExt(const SetpointVector& trajectory, const FrameID& frame_id, 
                              const QuadState& start_state, const Scalar duration, 
                              const std::string& name);
  ~TrajectoryExt() = default;

  void replaceFirstPoint(const TrajectoryExtPoint& first_point);
  void fitPolynomialCoeffs(const unsigned int poly_order,
                           const unsigned int continuity_order);
  void enableYawing(const bool enable_yawing);
  void resamplePointsFromPolyCoeffs();
  Eigen::Vector3d evaluatePoly(const double dt, const int derivative);

  bool setConstantArcLengthSpeed(const double &speed,
                                               const int &traj_len,
                                               const double &traj_dt);
  bool feasibilityResampling(const double &acc);
  void getTrajectory(SetpointVector* trajectory);
  using ReferenceBase::getSetpoint;
  Setpoint getSetpoint(const QuadState& state, const Scalar time) override;
  virtual Setpoint getStartSetpoint() override final;
  virtual Setpoint getEndSetpoint() override final;
  virtual bool isRLtrajectoryReference() const override { return true; }

 private:
  FrameID frame_id_;
  bool yawing_enabled_ = false;
  unsigned poly_order_;
  unsigned int continuity_order_;
  std::vector<Eigen::Vector3d> poly_coeff_;
  std::vector<TrajectoryExtPoint> points_;
};
}  // namespace agi