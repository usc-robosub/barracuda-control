#pragma once

#include <Eigen/Dense>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

namespace barracuda_control {

class AUVLQR {
public:
  AUVLQR(const Eigen::Matrix3d &M_t, const Eigen::Matrix3d &I_rot,
         const Eigen::Matrix3d &D_t, const Eigen::Matrix3d &D_r, double dt);

  void setCostMatrices(const Eigen::Matrix<double, 12, 12> &Q,
                       const Eigen::Matrix<double, 6, 6> &R);

  Eigen::Matrix<double, 6, 1>
  computeWrench(const Eigen::Matrix<double, 12, 1> &state,
                const Eigen::Matrix<double, 12, 1> &ref, double q0_state,
                double q0_ref);

private:
  void computeLinearizedSystemMatrix(const Eigen::Matrix<double, 12, 1> &ref,
                                     double q0_ref);
  void computeLinearizedInputMatrix();

  static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

  Eigen::Matrix3d M_t_;
  Eigen::Matrix3d I_rot_;
  Eigen::Matrix3d M_t_inv_;
  Eigen::Matrix3d I_rot_inv_;
  Eigen::Matrix3d D_t_;
  Eigen::Matrix3d D_r_;
  double dt_;

  Eigen::Matrix<double, 12, 12> A_;
  Eigen::Matrix<double, 12, 6> B_;
  Eigen::Matrix<double, 12, 12> Q_;
  Eigen::Matrix<double, 6, 6> R_;
  ct::core::FeedbackMatrix<12, 6> K_;
  bool B_initialized_;
};

} // namespace barracuda_control
