#include "agilib/simulator/bem/propeller_state.hpp"

namespace agi {

PropellerState::PropellerState(const BEMParameters& params) : param_(params) {}

void PropellerState::update(const Ref<const ArrayVector<QS::SIZE>>& state,
                            const Ref<const Matrix<3, 4>>& t_BM)

{
  // Extract motor speeds, angular rates and linear velocity
  omega_mot_ = state.segment<QS::NMOT>(QS::MOT).cwiseMax(10);
  vel_ = state.segment<QS::NVEL>(QS::VEL).array() * flu_conv_frd_;
  w_ = state.segment<QS::NOME>(QS::OME).array() * flu_conv_frd_;

  // Rotation matrix from body to inertial frame
  rot_ = Quaternion(state(QS::ATTW), state(QS::ATTX), state(QS::ATTY),
                    state(QS::ATTZ))
           .toRotationMatrix();

  // 1. Flow state around each propeller
  // note the minus sign due to a x b = -b x a
  velocity_ = (flu_conv_frd_.matrix().asDiagonal() * rot_.transpose() *
               state.segment<QS::NVEL>(QS::VEL).matrix())
                .replicate<1, 4>() -
              (t_BM.colwise().cross(w_));

  // Horizontal (in-plane) velocity
  vhor_ = velocity_.block<2, QS::NMOT>(0, 0).colwise().norm().array() + 1E-6;

  // Vertical (perpendicular to plane) velocity
  // downwards is positive
  vver_ = velocity_.block<1, QS::NMOT>(2, 0).array();

  // Total velocity
  vtot_ = velocity_.colwise().norm().array() + 1E-6;

  // Angle of attack of the propeller shaft
  alpha_s_ = vver_.array().binaryExpr(vhor_.array(), ApproxAtan2<Scalar>());

  // Advance Ratio of the propeller tip
  mu_ = vhor_.array() / (omega_mot_.array() * param_.r_prop_);

  // Disable Thrust Distortion
  K_ = Vector<QS::NMOT>::Zero();
  a0_.setZero();
  a1s_.setZero();
  b1s_.setZero();
}

std::ostream& operator<<(std::ostream& os, const PropellerState& s) {
  os.precision(3);
  os << std::scientific;
  os << "PropellerState: \n"
     << "Body Velocity          " << s.vel_.transpose() << "\n"
     << "Body Angular Rates     " << s.w_.transpose() << "\n"
     << "Propeller Speed        " << s.omega_mot_.transpose() << "\n"
     << "Propeller Air Velocity " << s.velocity_ << "\n"
     << "Horizontal Velocity    " << s.vhor_.transpose() << "\n"
     << "Vertical Velocity      " << s.vver_.transpose() << "\n"
     << "Induced Velocity       " << s.vind_.transpose() << "\n"
     << "Advance Ratio          " << s.mu_.transpose() << "\n"
     << "Coning Angle           " << s.a0_.transpose() << "\n"
     << "Lateral Flappling      " << s.b1s_.transpose() << "\n"
     << "Longitudinal Flapping  " << s.a1s_.transpose() << "\n"
     << "Thrust                 " << s.thrust_.transpose() << "\n"
     << "Torque                 " << s.torque_.transpose() << "\n"
     << "Hforce                 " << s.hforce_.transpose() << std::endl
     << s.param_ << std::endl;
  os.precision();
  os.unsetf(std::ios::scientific);
  return os;
}

void PropellerState::calculateFlapping() {
  Array<QS::NMOT, 7> mu = Array<QS::NMOT, 7>::Ones();
  Array<QS::NMOT, 7> omega_mot = Array<QS::NMOT, 7>::Ones();
  const Scalar p = w_[0];
  const Scalar q = w_[1];

  for (int i = 1; i < 7; ++i) {
    mu.col(i) *= mu.col(i - 1) * this->mu_.array();
    omega_mot.col(i) *= omega_mot.col(i - 1) * this->omega_mot_.array();
  }

  a0_ = (17860.69768 - 9.085245727e-8 * omega_mot.col(3) * mu.col(1) * q +
         5.075429983e-14 * omega_mot.col(5) * mu.col(2) * vind_.array() -
         1.201149295e-15 * omega_mot.col(5) * mu.col(3) * p -
         3.287355978e-15 * omega_mot.col(6) * alpha_s_ * mu.col(3) -
         8.012930184e-15 * omega_mot.col(6) * alpha_s_ * mu.col(1) +
         1.011494156e-15 * omega_mot.col(6) * alpha_s_ * mu.col(5) -
         8.680285403 * omega_mot.col(1) * mu.col(1) * p -
         17.36057081 * omega_mot.col(2) * alpha_s_ * mu.col(1) +
         1.517241217e-15 * omega_mot.col(5) * mu.col(5) * p -
         5.958332683e-15 * omega_mot.col(5) * mu.col(1) * p -
         1.561670740e-14 * omega_mot.col(5) * mu.col(4) * vind_.array() -
         3.121900413e-12 * omega_mot.col(4) * mu.col(4) +
         1.237136043e-13 * omega_mot.col(5) * vind_.array() -
         3.157428176e-15 * omega_mot.col(6) * mu.col(2) +
         6.274197178e-16 * omega_mot.col(6) * mu.col(6) -
         2.031302206e-16 * omega_mot.col(6) * mu.col(4) +
         268.0341334 * omega_mot.col(1) * vind_.array() -
         3.589529587 * omega_mot.col(2) * mu.col(2) -
         1.902162223e-15 * omega_mot.col(6) +
         8.243768249e-12 * omega_mot.col(4) - 4.121166794 * omega_mot.col(2)) /
        (-4.377059028e8 - 1.147609987e-7 * omega_mot.col(4) * mu.col(2) +
         7.650732730e-8 * omega_mot.col(4) * mu.col(4) +
         1.135865755e-14 * omega_mot.col(6) * mu.col(4) -
         2.999395522e-14 * omega_mot.col(6) -
         2.020271598e-7 * omega_mot.col(4) - 64.98399099 * omega_mot.col(2));

  b1s_ = 3.034482349e-15 * omega_mot.col(1) *
         (14.44639117 * omega_mot.col(4) * q +
          0.6202900305 * omega_mot.col(5) * mu.col(5) +
          1.177725817e17 * mu.col(1) * vind_.array() -
          1.208793256 * omega_mot.col(5) * mu.col(3) -
          1.157259683 * omega_mot.col(5) * mu.col(1) +
          9.730505319e7 * omega_mot.col(2) * q -
          3.482171723e15 * omega_mot.col(1) * mu.col(1) -
          3086.424696 * omega_mot.col(3) * mu.col(3) -
          5.169754829e8 * omega_mot.col(3) * mu.col(1) -
          6.724278493e8 * omega_mot.col(2) * p -
          5.988003267e7 * omega_mot.col(2) * mu.col(2) * q +
          1.500000041 * omega_mot.col(4) * mu.col(4) * p -
          8.890086873 * omega_mot.col(4) * mu.col(2) * q -
          7.628130118e15 * omega_mot.col(1) * alpha_s_ * mu.col(2) -
          4.529202255e15 * p -
          15.43924687 * omega_mot.col(4) * mu.col(3) * vind_.array() +
          1.748510208e10 * omega_mot.col(2) * mu.col(1) * vind_.array() +
          75.26632912 * omega_mot.col(4) * mu.col(1) * vind_.array() +
          0.9999999999 * omega_mot.col(5) * alpha_s_ * mu.col(4) -
          4.875000134 * omega_mot.col(5) * alpha_s_ * mu.col(2) -
          3.625000097 * omega_mot.col(4) * mu.col(2) * p -
          1.132510062e9 * omega_mot.col(3) * alpha_s_ * mu.col(2)) /
         (-4.377059028e8 - 1.147609987e-7 * omega_mot.col(4) * mu.col(2) +
          7.650732720e-8 * omega_mot.col(4) * mu.col(4) +
          1.135865756e-14 * omega_mot.col(6) * mu.col(4) -
          2.999395520e-14 * omega_mot.col(6) -
          2.020271601e-7 * omega_mot.col(4) - 64.98399099 * omega_mot.col(2));

  a1s_ = 4.543463022e-14 *
         (0.4564908655 * omega_mot.col(5) * mu.col(3) +
          0.7417976561 * omega_mot.col(5) * mu.col(1) +
          4.491002632e7 * omega_mot.col(2) * q +
          0.9648437489 * omega_mot.col(4) * p +
          1.559168274e10 * omega_mot.col(1) * mu.col(1) -
          58779.37391 * omega_mot.col(3) * mu.col(3) +
          1.398833618e6 * omega_mot.col(3) * mu.col(1) +
          6.498797591e6 * omega_mot.col(2) * p + 3.024957887e14 * q -
          15.43924657 * omega_mot.col(4) * mu.col(3) * vind_.array() -
          3.578285830e6 * omega_mot.col(2) * mu.col(2) * p +
          6.499534432e7 * omega_mot.col(2) * mu.col(1) * vind_.array() -
          25.08877564 * omega_mot.col(4) * mu.col(1) * vind_.array() -
          1.039925259e8 * omega_mot.col(2) * mu.col(3) * vind_.array() +
          omega_mot.col(5) * alpha_s_ * mu.col(4) +
          1.625000001 * omega_mot.col(5) * alpha_s_ * mu.col(2) +
          0.5937500002 * omega_mot.col(4) * mu.col(2) * p +
          6.735595891e6 * omega_mot.col(3) * alpha_s_ * mu.col(4) -
          4.209748462e6 * omega_mot.col(3) * alpha_s_ * mu.col(2)) *
         omega_mot.col(1) /
         (-4.377059028e8 - 1.147609987e-7 * omega_mot.col(4) * mu.col(2) +
          7.650732720e-8 * omega_mot.col(4) * mu.col(4) +
          1.135865755e-14 * omega_mot.col(6) * mu.col(4) -
          2.999395520e-14 * omega_mot.col(6) -
          2.020271610e-7 * omega_mot.col(4) - 64.98399099 * omega_mot.col(2));
}

}  // namespace agi
