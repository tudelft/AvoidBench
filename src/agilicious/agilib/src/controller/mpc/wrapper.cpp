#include "agilib/controller/mpc/wrapper.hpp"

#include <iostream>

namespace agi {

namespace acados {
MpcWrapper::MpcWrapper()
  : online_params_((Vector<NP>() << 0.752,  // initial mass
                    1.0, 0.0, 0.0, 0.0,     // initial quat reference
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // initial cds
                    0.075, -0.075, -0.075, 0.075,       // Initial lxs
                    -0.1, 0.1, -0.1, 0.1,               // Initial lys
                    0.022,                              // kappa
                    0.0025, 0.0021, 0.0043)             // inertias
                     .finished()
                     .replicate(1, N + 1)),
    lbx_((Vector<NBX>() << -10.0, -10.0, -4.0)  // initial z, omega lb
           .finished()
           .replicate(1, N)),
    ubx_((Vector<NBX>() << 10.0, 10.0, 4.0)  // initial z, omega ub
           .finished()
           .replicate(1, N)),
    lbu_((Vector<NBU>() << 0.0, 0.0, 0.0, 0.0  // initial thrusts lb
          )
           .finished()
           .replicate(1, N)),
    ubu_((Vector<NBU>() << 8.5, 8.5, 8.5, 8.5  // initial thrusts ub
          )
           .finished()
           .replicate(1, N)),
    dt_(0.05),
    logger_("wrapper") {
  p_acados_ocp_capsule_ = drone_model_acados_create_capsule();
  const int status = drone_model_acados_create(p_acados_ocp_capsule_);
  if (status) {
    logger_.fatal("Could not create acados! Returned %d", status);
  }

  p_nlp_config_ = drone_model_acados_get_nlp_config(p_acados_ocp_capsule_);
  p_nlp_dims_ = drone_model_acados_get_nlp_dims(p_acados_ocp_capsule_);
  p_nlp_in_ = drone_model_acados_get_nlp_in(p_acados_ocp_capsule_);
  p_nlp_out_ = drone_model_acados_get_nlp_out(p_acados_ocp_capsule_);
  p_nlp_solver_ = drone_model_acados_get_nlp_solver(p_acados_ocp_capsule_);
  p_nlp_opts_ = drone_model_acados_get_nlp_opts(p_acados_ocp_capsule_);

  dt_ = p_nlp_in_->Ts[0];
}

MpcWrapper::~MpcWrapper() {
  // Free solver
  const int status_free_model = drone_model_acados_free(p_acados_ocp_capsule_);
  if (status_free_model) {
    logger_.info("drone_model_acados_free() returned status %d. \n",
                 status_free_model);
  }
  // Free capsule
  const int status_free_capsule =
    drone_model_acados_free_capsule(p_acados_ocp_capsule_);
  if (status_free_capsule) {
    logger_.info("drone_model_acados_free_capsule() returned status %d. \n",
                 status_free_capsule);
  }
}

void MpcWrapper::setInitialState(const Vector<NX>& state) {
  setStateConstraint(state, state, 0);
}

void MpcWrapper::setBodyRateConstraints(const Vector<3>& lower_bound,
                                        const Vector<3>& upper_bound) {
  lbx_.block<3, N>(STATECONSTR::OMEGA_X, 0) = lower_bound.replicate(1, N);
  ubx_.block<3, N>(STATECONSTR::OMEGA_X, 0) = upper_bound.replicate(1, N);
}

void MpcWrapper::setThrustConstraints(const Vector<NU>& lower_bound,
                                      const Vector<NU>& upper_bound) {
  lbu_.block<4, N>(INPUTCONSTR::T_0, 0) = lower_bound.replicate(1, N);
  ubu_.block<4, N>(INPUTCONSTR::T_0, 0) = upper_bound.replicate(1, N);
}

void MpcWrapper::updateInputConstraints() {
  for (int i = 0; i < N; ++i) {
    setInputConstraint(lbu_.col(i), ubu_.col(i), i);
  }
}

void MpcWrapper::updateStateConstraints() {
  for (int i = 0; i < N; ++i) {
    setStateConstraint(lbx_.col(i), ubx_.col(i), i + 1, constr_idx_x_);
  }
}

void MpcWrapper::setStateConstraint(const Vector<>& state_lb,
                                    const Vector<>& state_ub, const int index,
                                    const VectorInt<>& indexes_constr) {
  if (index < 0 || index > N) {
    std::cout << "State constraint index out of bounds" << std::endl;
    return;
  }
  assert(state_lb.size() == state_ub.size());
  assert(state_lb.size() == indexes_constr.size());
  const int n_constraints = state_lb.size();

  double lbx[n_constraints];
  double ubx[n_constraints];
  int idxbx[n_constraints];

  Map<Vector<Dynamic>>(&lbx[0], n_constraints, 1) = state_lb;
  Map<Vector<Dynamic>>(&ubx[0], n_constraints, 1) = state_ub;
  Map<VectorInt<Dynamic>>(&idxbx[0], n_constraints, 1) = indexes_constr;


  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "idxbx", idxbx);

  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "lbx", lbx);
  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "ubx", ubx);
}

void MpcWrapper::setInputConstraint(const Vector<>& input_lb,
                                    const Vector<>& input_ub, const int index,
                                    const VectorInt<>& indexes_constr) {
  if (index < 0 || index >= N) {
    std::cout << "Input constraint index out of bounds" << std::endl;
    return;
  }
  assert(input_lb.size() == input_ub.size());
  assert(input_lb.size() == indexes_constr.size());
  const int n_constraints = input_lb.size();

  double lbu[n_constraints];
  double ubu[n_constraints];
  int idxbu[n_constraints];

  Map<Vector<Dynamic>>(&lbu[0], n_constraints, 1) = input_lb;
  Map<Vector<Dynamic>>(&ubu[0], n_constraints, 1) = input_ub;
  Map<VectorInt<Dynamic>>(&idxbu[0], n_constraints, 1) = indexes_constr;


  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "idxbu", idxbu);

  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "lbu", lbu);
  ocp_nlp_constraints_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, index,
                                "ubu", ubu);
}

int MpcWrapper::update(const Vector<NX>& state) {
  updateOnlineParams();  // First of all, update online parameters
  updateInputConstraints();
  updateStateConstraints();
  // In case we would want to separate separating and feedback phases:
  int rti_phase = 0;  // 1-> preparation, 2-> feedback, 0->both
  ocp_nlp_solver_opts_set(p_nlp_config_, p_nlp_opts_, "rti_phase", &rti_phase);

  // Set next state
  setInitialState(state);

  // solve
  const int status = drone_model_acados_solve(p_acados_ocp_capsule_);
  double x_pred[NX * (N + 1)];
  double u_pred[NU * N];

  for (int ii = 0; ii < N + 1; ii++) {
    ocp_nlp_out_get(p_nlp_config_, p_nlp_dims_, p_nlp_out_, ii, "x",
                    &x_pred[ii * NX]);
    if (ii < N) {
      ocp_nlp_out_get(p_nlp_config_, p_nlp_dims_, p_nlp_out_, ii, "u",
                      &u_pred[ii * NU]);
    }
  }

  // Update state and input predictions
  states_pred_ = Map<Matrix<NX, N + 1>>(&x_pred[0], NX, N + 1);
  inputs_pred_ = Map<Matrix<NU, N>>(&u_pred[0], NU, N);

  int sqp_iter;
  double time_lin, time_qp_sol, time_tot;

  ocp_nlp_get(p_nlp_config_, p_nlp_solver_, "sqp_iter", &sqp_iter);
  ocp_nlp_get(p_nlp_config_, p_nlp_solver_, "time_tot", &time_tot);
  ocp_nlp_get(p_nlp_config_, p_nlp_solver_, "time_qp_sol", &time_qp_sol);
  ocp_nlp_get(p_nlp_config_, p_nlp_solver_, "time_lin", &time_lin);

  // printf(
  //   "iters %d, time (total %f, lineariz. %f, qp_sol. %f) "
  //   "ms\n",
  //   sqp_iter, time_tot * 1e3, time_lin * 1e3, time_qp_sol * 1e3);

  return status;
}

void MpcWrapper::setReferences(const Matrix<NX, N>& state_refs,
                               const Matrix<NU, N>& input_refs) {
  for (int ii = 0; ii < N; ++ii) {
    setReference(state_refs.col(ii), input_refs.col(ii), ii);
  }
}

void MpcWrapper::setReference(const Vector<NX>& state_ref,
                              const Vector<NU>& input_ref, const int i) {
  double y_ref[NY];
  Vector<NQ> state_ref_corrected;
  state_ref_corrected.segment<3>(REFIDX::REFPOS) =
    state_ref.segment<3>(STATEIDX::STATEPOS);
  state_ref_corrected.segment<3>(REFIDX::REFATT) = Vector<3>::Zero();
  state_ref_corrected.segment<3>(REFIDX::REFVEL) =
    state_ref.segment<3>(STATEIDX::STATEVEL);
  state_ref_corrected.segment<3>(REFIDX::REFOME) =
    state_ref.segment<3>(STATEIDX::STATEOME);
  setQuaternionReference(state_ref.segment<4>(STATEIDX::STATEATT), i);

  Map<Vector<NQ>>(&y_ref[0], NQ, 1) = state_ref_corrected;
  Map<Vector<NU>>(&y_ref[NQ], NU, 1) = input_ref;

  ocp_nlp_cost_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, i, "yref",
                         &y_ref);
}

void MpcWrapper::setReferenceN(const Vector<NX>& state_ref_N) {
  double y_ref_N[NYN];
  Vector<NQ> state_ref_N_corrected;
  state_ref_N_corrected.segment<3>(REFIDX::REFPOS) =
    state_ref_N.segment<3>(STATEIDX::STATEPOS);
  state_ref_N_corrected.segment<3>(REFIDX::REFATT) = Vector<3>::Zero();
  state_ref_N_corrected.segment<3>(REFIDX::REFVEL) =
    state_ref_N.segment<3>(STATEIDX::STATEVEL);
  state_ref_N_corrected.segment<3>(REFIDX::REFOME) =
    state_ref_N.segment<3>(STATEIDX::STATEOME);
  setQuaternionReference(state_ref_N.segment<4>(STATEIDX::STATEATT), N);
  Map<Vector<NQ>>(&y_ref_N[0], NQ, 1) = state_ref_N_corrected;
  ocp_nlp_cost_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, N, "yref",
                         &y_ref_N);
}

void MpcWrapper::setCosts(const Matrix<NY, NY * N> Ws, const Scalar gamma) {
  for (int ii = 0; ii < N; ++ii) {
    setCost(pow(gamma, ii) * Ws.block<NY, NY>(0, ii * NY), ii);
  }
}

void MpcWrapper::setCostN(const Matrix<NYN, NYN>& W_N, const Scalar gamma) {
  double wn_c[NYN * NYN];
  Map<Matrix<NYN, NYN>>(&wn_c[0], NYN, NYN) = W_N * pow(gamma, N);
  ocp_nlp_cost_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, N, "W", wn_c);
}

void MpcWrapper::setCost(const Matrix<NY, NY>& W, const int i) {
  double w_c[NY * NY];
  Map<Matrix<NY, NY>>(&w_c[0], NY, NY) = W;
  ocp_nlp_cost_model_set(p_nlp_config_, p_nlp_dims_, p_nlp_in_, i, "W", w_c);
}

void MpcWrapper::setQuaternionReference(const Vector<4>& q_ref, const int i) {
  online_params_.block<4, 1>(ONLPARAMIDX::Q_REF_0, i) = q_ref;
}

void MpcWrapper::setMass(const Vector<N + 1>& mass) {
  online_params_.row(ONLPARAMIDX::MASS) = mass;
}

void MpcWrapper::setAeroCoeffs(const Matrix<3, N + 1>& aero_coeff_1,
                               const Matrix<3, N + 1>& aero_coeff_3,
                               const Vector<N + 1>& aero_coeff_h) {
  online_params_.block<3, N + 1>(ONLPARAMIDX::CDX1, 0) = aero_coeff_1;
  online_params_.block<3, N + 1>(ONLPARAMIDX::CDX3, 0) = aero_coeff_3;
  online_params_.row(ONLPARAMIDX::CDZH) = aero_coeff_h;
}

void MpcWrapper::setArmLengths(const Matrix<3, 4>& t_BM) {
  online_params_.block<4, N + 1>(ONLPARAMIDX::L_X_0, 0) =
    t_BM.row(0).transpose().replicate(1, N + 1);
  online_params_.block<4, N + 1>(ONLPARAMIDX::L_Y_0, 0) =
    t_BM.row(1).transpose().replicate(1, N + 1);
}

void MpcWrapper::setKappa(const Vector<N + 1>& kappa) {
  online_params_.row(ONLPARAMIDX::KAPPA) = kappa;
}

void MpcWrapper::setInertia(const Vector<3>& inertia) {
  online_params_.block<3, N + 1>(ONLPARAMIDX::I_XX, 0) =
    inertia.replicate(1, N + 1);
}


void MpcWrapper::updateOnlineParams() {
  double params_c[NP * (N + 1)];
  Map<Matrix<NP, N + 1>>(&params_c[0], NP, N + 1) = online_params_;

  for (int ii = 0; ii < N + 1; ++ii) {
    drone_model_acados_update_params(p_acados_ocp_capsule_, ii,
                                     &params_c[ii * NP], NP);
  }
}
}  // namespace acados
}  // namespace agi
