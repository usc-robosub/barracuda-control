#include "barracuda_control/auv_lqr.hpp"
#include <Eigen/Geometry>

namespace barracuda_control {

AUVLQR::AUVLQR(const Eigen::Matrix3d &M_t, const Eigen::Matrix3d &I_rot,
               const Eigen::Matrix3d &D_t, const Eigen::Matrix3d &D_r,
               double dt)
    : M_t_(M_t), I_rot_(I_rot), D_t_(D_t), D_r_(D_r), dt_(dt),
      B_initialized_(false) {
  M_t_inv_ = M_t_.inverse();
  I_rot_inv_ = I_rot_.inverse();
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

Eigen::Matrix3d AUVLQR::skewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

void AUVLQR::computeLinearizedInputMatrix() {
  B_.setZero();
  B_.block<3, 3>(6, 0) = M_t_inv_;
  B_.block<3, 3>(9, 3) = I_rot_inv_;
  B_initialized_ = true;
}

void AUVLQR::computeLinearizedSystemMatrix(
    const Eigen::Matrix<double, 12, 1> &ref, double q0_ref) {
  Eigen::Quaterniond q_ref(q0_ref, ref(3), ref(4), ref(5));
  Eigen::Matrix3d R_ib = q_ref.toRotationMatrix();

  Eigen::Vector3d v_ref = ref.segment<3>(6);
  Eigen::Vector3d w_ref = ref.segment<3>(9);

  A_.setZero();
  A_.block<3, 3>(0, 6) = R_ib; // position derivative
  A_.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity();
  A_.block<3, 3>(6, 6) = -M_t_inv_ * D_t_ - skewSymmetric(w_ref);
  A_.block<3, 3>(6, 9) = -M_t_inv_ * skewSymmetric(v_ref);
  A_.block<3, 3>(9, 9) = -I_rot_inv_ *
                         (D_r_ + skewSymmetric(I_rot_ * w_ref) -
                          skewSymmetric(w_ref) * I_rot_);

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
