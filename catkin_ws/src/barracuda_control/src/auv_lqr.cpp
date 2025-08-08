#include "barracuda_control/auv_lqr.hpp"

namespace barracuda_control {

AUVLQR::AUVLQR(double mass, const Eigen::Matrix3d &inertia,
               const Eigen::Matrix3d &D_t, const Eigen::Matrix3d &D_r,
               double dt)
    : mass_(mass), inertia_(inertia), D_t_(D_t), D_r_(D_r), dt_(dt),
      B_initialized_(false) {
  A_.setZero();
  B_.setZero();
  Q_.setZero();
  R_.setZero();
}

void AUVLQR::setCostMatrices(const Eigen::Matrix<double, 12, 12> &Q,
                             const Eigen::Matrix<double, 6, 6> &R) {
  Q_ = Q;
  R_ = R;
}

void AUVLQR::computeLinearizedInputMatrix() {
  B_.setZero();
  B_.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity() / mass_;
  B_.block<3, 3>(9, 3) = inertia_.inverse();
  B_initialized_ = true;
}

void AUVLQR::computeLinearizedSystemMatrix(
    const Eigen::Matrix<double, 12, 1> &ref, double q0_ref) {
  Eigen::Quaterniond q_ref(q0_ref, ref(3), ref(4), ref(5));
  Eigen::Matrix3d R_ib = q_ref.toRotationMatrix();

  A_.setZero();
  A_.block<3, 3>(0, 6) = R_ib; // position derivative
  A_.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity();
  A_.block<3, 3>(6, 6) = -D_t_ / mass_;              // linear velocity damping
  A_.block<3, 3>(9, 9) = -inertia_.inverse() * D_r_; // angular velocity damping

  if (!B_initialized_)
    computeLinearizedInputMatrix();
}

Eigen::Matrix<double, 6, 1>
AUVLQR::computeWrench(const Eigen::Matrix<double, 12, 1> &state,
                      const Eigen::Matrix<double, 12, 1> &ref, double q0_state,
                      double q0_ref) {
  computeLinearizedSystemMatrix(ref, q0_ref);

  Eigen::Quaterniond q_state(q0_state, state(3), state(4), state(5));
  Eigen::Quaterniond q_ref(q0_ref, ref(3), ref(4), ref(5));
  Eigen::Quaterniond q_error = q_ref * q_state.conjugate();

  Eigen::Matrix<double, 12, 1> error;
  error.head<3>() = state.head<3>() - ref.head<3>();
  error.segment<3>(3) = -q_error.vec();
  error.segment<3>(6) = state.segment<3>(6) - ref.segment<3>(6);
  error.segment<3>(9) = state.segment<3>(9) - ref.segment<3>(9);

  ct::optcon::LQR<12, 6> solver;
  solver.compute(Q_, R_, A_, B_, K_);
  return -K_ * error;
}

} // namespace barracuda_control
