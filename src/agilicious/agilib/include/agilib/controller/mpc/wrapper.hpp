#pragma once


#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {
namespace acados {

#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "agilib/controller/mpc/acados/acados_solver_drone_model.h"

class MpcWrapper {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  MpcWrapper();
  ~MpcWrapper();


  static constexpr int NX = DRONE_MODEL_NX;
  static constexpr int NZ = DRONE_MODEL_NZ;
  static constexpr int NU = DRONE_MODEL_NU;
  static constexpr int NP = DRONE_MODEL_NP;
  static constexpr int NBX = DRONE_MODEL_NBX;
  static constexpr int NBX0 = DRONE_MODEL_NBX0;
  static constexpr int NBU = DRONE_MODEL_NBU;
  static constexpr int NSBX = DRONE_MODEL_NSBX;
  static constexpr int NSBU = DRONE_MODEL_NSBU;
  static constexpr int NSH = DRONE_MODEL_NSH;
  static constexpr int NSG = DRONE_MODEL_NSG;
  static constexpr int NSPHI = DRONE_MODEL_NSPHI;
  static constexpr int NSHN = DRONE_MODEL_NSHN;
  static constexpr int NSGN = DRONE_MODEL_NSGN;
  static constexpr int NSPHIN = DRONE_MODEL_NSPHIN;
  static constexpr int NSBXN = DRONE_MODEL_NSBXN;
  static constexpr int NS = DRONE_MODEL_NS;
  static constexpr int NSN = DRONE_MODEL_NSN;
  static constexpr int NG = DRONE_MODEL_NG;
  static constexpr int NBXN = DRONE_MODEL_NBXN;
  static constexpr int NGN = DRONE_MODEL_NGN;
  static constexpr int NY0 = DRONE_MODEL_NY0;
  static constexpr int NY = DRONE_MODEL_NY;
  static constexpr int NYN = DRONE_MODEL_NYN;
  static constexpr int N = DRONE_MODEL_N;
  static constexpr int NH = DRONE_MODEL_NH;
  static constexpr int NPHI = DRONE_MODEL_NPHI;
  static constexpr int NHN = DRONE_MODEL_NHN;
  static constexpr int NPHIN = DRONE_MODEL_NPHIN;
  static constexpr int NR = DRONE_MODEL_NR;

  static constexpr int NQ = NY - NU;

  void setInitialState(const Vector<NX> &state);
  int update(const Vector<NX> &state);
  void setStateConstraint(
    const Vector<> &state_lb, const Vector<> &state_ub, const int index,
    const VectorInt<> &indexes_constr =
      (VectorInt<NX>() << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12).finished());

  void setBodyRateConstraints(const Vector<3> &lower_bound,
                              const Vector<3> &upper_bound);

  void setInputConstraint(const Vector<> &input_lb, const Vector<> &input_ub,
                          const int index,
                          const VectorInt<> &indexes_constr =
                            (VectorInt<NU>() << 0, 1, 2, 3).finished());

  void setThrustConstraints(const Vector<NU> &lower_bound,
                            const Vector<NU> &upper_bound);


  inline const Vector<NX> getState(const int i) const {
    return states_pred_.col(i);
  }
  inline const Matrix<NX, N + 1> getStates() const { return states_pred_; }

  inline const Vector<NU> getInput(const int i) const {
    return inputs_pred_.col(i);
  }
  inline const Matrix<NU, N> getInputs() const { return inputs_pred_; }


  void setReference(const Vector<NX> &state_ref, const Vector<NU> &input_ref,
                    const int i);
  void setReferences(const Matrix<NX, N> &state_refs,
                     const Matrix<NU, N> &input_refs);
  void setReferenceN(const Vector<NX> &state_ref_N);

  void setCosts(const Matrix<NY, NY * N> Ws, const Scalar gamma = 1.0);
  void setCost(const Matrix<NY, NY> &W, const int i);
  void setCostN(const Matrix<NYN, NYN> &W_N, const Scalar gamma = 1.0);
  void setMass(const Vector<N + 1> &mass);
  void setAeroCoeffs(const Matrix<3, N + 1> &aero_coeff_1,
                     const Matrix<3, N + 1> &aero_coeff_3,
                     const Vector<N + 1> &aero_coeff_h);
  void setArmLengths(const Matrix<3, 4> &t_BM);
  void setKappa(const Vector<N + 1> &kappa);
  void setInertia(const Vector<3> &inertia);

  inline void setStatesPred(const Matrix<NX, N + 1> states) {
    states_pred_ = states;
  }

  inline void setInputsPred(const Matrix<NU, N> inputs) {
    inputs_pred_ = inputs;
  }
  inline Scalar getDt() { return dt_; }

 private:
  drone_model_solver_capsule *p_acados_ocp_capsule_;
  ocp_nlp_config *p_nlp_config_;
  ocp_nlp_dims *p_nlp_dims_;
  ocp_nlp_in *p_nlp_in_;
  ocp_nlp_out *p_nlp_out_;
  ocp_nlp_solver *p_nlp_solver_;
  void *p_nlp_opts_;

  Matrix<NX, N + 1> states_pred_;
  Matrix<NU, N> inputs_pred_;
  Matrix<NP, N + 1> online_params_;

  Matrix<NBX, N> lbx_;  // From 1 to N+1
  Matrix<NBX, N> ubx_;  // From 1 to N+1
  Matrix<NBU, N> lbu_;  // From 0 to N
  Matrix<NBU, N> ubu_;  // From 0 to N

  const VectorInt<NBX> constr_idx_x_ =
    (VectorInt<NBX>() << 10, 11, 12).finished();
  const VectorInt<NBU> constr_idx_u_ =
    (VectorInt<NBU>() << 0, 1, 2, 3).finished();

  void updateInputConstraints();
  void updateStateConstraints();

  Scalar dt_;

  void updateOnlineParams();
  void setQuaternionReference(const Vector<4> &q_ref, const int i);

  Logger logger_;

 public:
  enum STATECONSTR : int {
    OMEGA_X = 0,
    OMEGA_Y = 1,
    OMEGA_Z = 2,
  };

  enum INPUTCONSTR : int { T_0 = 0, T_1 = 1, T_2 = 2, T_3 = 3 };

  enum ONLPARAMIDX : int {
    MASS = 0,
    Q_REF_0 = 1,
    Q_REF_1 = 2,
    Q_REF_2 = 3,
    Q_REF_3 = 4,
    CDX1 = 5,
    CDY1 = 6,
    CDZ1 = 7,
    CDX3 = 8,
    CDY3 = 9,
    CDZ3 = 10,
    CDZH = 11,
    L_X_0 = 12,
    L_X_1 = 13,
    L_X_2 = 14,
    L_X_3 = 15,
    L_Y_0 = 16,
    L_Y_1 = 17,
    L_Y_2 = 18,
    L_Y_3 = 19,
    KAPPA = 20,
    I_XX = 21,
    I_YY = 22,
    I_ZZ = 23
  };

  enum STATEIDX : int {
    STATEPOS = 0,
    STATEATT = 3,
    STATEVEL = 7,
    STATEOME = 10
  };

  enum REFIDX : int {
    REFPOS = 0,
    REFATT = 3,
    REFVEL = 6,
    REFOME = 9,
    REFU = 12
  };
};

}  // namespace acados
}  // namespace agi
