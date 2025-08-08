#include "barracuda_control/SetThrustZero.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ct/optcon/optcon.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vector>

#define STATE_DIM 12
#define CONTROL_DIM 6

class LqrNode {
public:
  static double X_[STATE_DIM];
  static double X0_[STATE_DIM];
  static Eigen::Quaterniond target_quat_;
  int rate;
  bool thrust_zero_enabled;
  double dt_;

  LqrNode(ros::NodeHandle &nh) : nh_(nh), thrust_zero_enabled(false) {
    ROS_INFO("Discrete-time LQR Node initialized and running.");
    double dt;
    nh.getParam("lqr/update_rate", rate);
    // Optional control smoothing in [0,1], where 1.0 = no smoothing
    nh.param("lqr/control_smoothing_alpha", smoothing_alpha_, 1.0);
    if (smoothing_alpha_ < 0.0)
      smoothing_alpha_ = 0.0;
    if (smoothing_alpha_ > 1.0)
      smoothing_alpha_ = 1.0;
    dt = 1.0 / rate;
    dt_ = dt;

    // Define basic identity matrix for convenience
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // Vehicle physical parameters. Prefer full 6x6 model if provided.
    // Fallback uses block-diagonal 3x3 matrices.
    Eigen::Matrix3d M_t = Eigen::Matrix3d::Identity();   // translational mass
    Eigen::Matrix3d I_rot = Eigen::Matrix3d::Identity(); // rotational inertia
    Eigen::Matrix3d D_t = Eigen::Matrix3d::Zero();       // translational damping
    Eigen::Matrix3d D_r = Eigen::Matrix3d::Zero();       // rotational damping
    getRosParamMatrix(nh, "lqr/M_t", M_t, true);
    getRosParamMatrix(nh, "lqr/I_rot", I_rot, true);
    getRosParamMatrix(nh, "lqr/D_t", D_t, false);
    getRosParamMatrix(nh, "lqr/D_r", D_r, false);

    Eigen::Matrix<double, 6, 6> M6 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> D6 = Eigen::Matrix<double, 6, 6>::Zero();
    bool have_M6 = getRosParamMatrix6(nh, "lqr/M6", M6);
    bool have_D6 = getRosParamMatrix6(nh, "lqr/D6", D6);

    if (!have_M6) {
      M6.topLeftCorner<3, 3>() = M_t;
      M6.bottomRightCorner<3, 3>() = I_rot;
      ROS_INFO("Using block-diagonal mass/inertia (no lqr/M6 provided)");
    } else {
      ROS_INFO("Using full 6x6 mass/inertia from lqr/M6");
    }

    if (!have_D6) {
      D6.topLeftCorner<3, 3>() = D_t;
      D6.bottomRightCorner<3, 3>() = D_r;
      ROS_INFO("Using block-diagonal damping (no lqr/D6 provided)");
    } else {
      ROS_INFO("Using full 6x6 damping from lqr/D6");
    }

    // Precompute inverse of the 6x6 mass/inertia (assumed invertible)
    Eigen::Matrix<double, 6, 6> M6_inv = M6.inverse();

    // Build discrete-time state-space matrices directly.
    // State ordering: [ p(0-2); theta(3-5); v(6-8); omega(9-11) ]
    Eigen::MatrixXd A_d = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(STATE_DIM, CONTROL_DIM);

    // Position update (initialized assuming small angles):
    // p[k+1] = p[k] + dt * R(q)^{} v_body[k]; we initialize with Identity and
    // update with the current rotation each cycle in computeLqr().
    A_d.block<3, 3>(0, 0) = I3;
    A_d.block<3, 3>(0, 6) = dt * I3;

    // Orientation update: theta[k+1] = theta[k] + dt * omega[k]
    A_d.block<3, 3>(3, 3) = I3;
    A_d.block<3, 3>(3, 9) = dt * I3;

    // Combined body velocity update (v, omega):
    // V6[k+1] = (I6 - dt * M6^{-1} * D6) * V6[k] + dt * M6^{-1} * U[k]
    Eigen::Matrix<double, 6, 6> Avv =
        Eigen::Matrix<double, 6, 6>::Identity() - dt * (M6_inv * D6);
    A_d.block<3, 3>(6, 6) = Avv.topLeftCorner<3, 3>();
    A_d.block<3, 3>(6, 9) = Avv.topRightCorner<3, 3>();
    A_d.block<3, 3>(9, 6) = Avv.bottomLeftCorner<3, 3>();
    A_d.block<3, 3>(9, 9) = Avv.bottomRightCorner<3, 3>();

    // Input matrix (maps 6D wrench to V6 update)
    Eigen::Matrix<double, 6, 6> Bv = dt * M6_inv;
    B_d.block<3, 3>(6, 0) = Bv.topLeftCorner<3, 3>();
    B_d.block<3, 3>(6, 3) = Bv.topRightCorner<3, 3>();
    B_d.block<3, 3>(9, 0) = Bv.bottomLeftCorner<3, 3>();
    B_d.block<3, 3>(9, 3) = Bv.bottomRightCorner<3, 3>();

    ROS_INFO_STREAM("Discrete-time A matrix:\n" << A_d);
    ROS_INFO_STREAM("Discrete-time B matrix:\n" << B_d);

    // Load the weighting matrices Q and R from ROS parameters.
    Eigen::VectorXd Q_vector = Eigen::VectorXd(STATE_DIM);
    Eigen::VectorXd R_vector = Eigen::VectorXd(CONTROL_DIM);
    getRosParamVector(nh, "lqr/Q", Q_vector, STATE_DIM);
    getRosParamVector(nh, "lqr/R", R_vector, CONTROL_DIM);
    // Store the computed matrices for LQR synthesis
    A_mat = A_d;
    B_mat = B_d;
    Q_mat = Q_vector.asDiagonal();
    R_mat = R_vector.asDiagonal();

    // Initialize feedback gain matrix
    K_ = ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero();

    // Set up ROS subscribers and publishers.
    // Match PID node's odometry topic
    odometry_sub = nh.subscribe("odometry/filtered/global", 1, odometryCallback);
    target_odometry_sub =
        nh.subscribe("target_odometry", 1, targetOdometryCallback);
    // Publish the control input
    control_pub =
        nh.advertise<geometry_msgs::Wrench>("thruster_manager/input", 1);

    // Set up service server for thrust zero control
    thrust_zero_service = nh.advertiseService(
        "set_thrust_zero", &LqrNode::setThrustZeroCallback, this);
  }

  void run() // main loop
  {
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
      computeLqr();
      publishControl();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void publishControl() {
    geometry_msgs::Wrench control_msg;

    if (thrust_zero_enabled) {
      // Set all forces and torques to zero
      control_msg.force.x = 0.0;
      control_msg.force.y = 0.0;
      control_msg.force.z = 0.0;
      control_msg.torque.x = 0.0;
      control_msg.torque.y = 0.0;
      control_msg.torque.z = 0.0;
    } else {
      // Normal LQR control computation
      Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd>(X_, STATE_DIM);
      Eigen::VectorXd X0 = Eigen::Map<Eigen::VectorXd>(X0_, STATE_DIM);
      Eigen::VectorXd U = -K_ * (X - X0);
      if (smoothing_alpha_ < 1.0) {
        if (U_prev_.size() != CONTROL_DIM)
          U_prev_ = Eigen::VectorXd::Zero(CONTROL_DIM);
        U = smoothing_alpha_ * U + (1.0 - smoothing_alpha_) * U_prev_;
        U_prev_ = U;
      }
      control_msg.force.x = U[0];
      control_msg.force.y = U[1];
      control_msg.force.z = U[2];
      control_msg.torque.x = U[3];
      control_msg.torque.y = U[4];
      control_msg.torque.z = U[5];
    }

    control_pub.publish(control_msg);
  }

  static void targetOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Set target position (indices 0-2)
    X0_[0] = msg->pose.pose.position.x;
    X0_[1] = msg->pose.pose.position.y;
    X0_[2] = msg->pose.pose.position.z;

    // Store target orientation quaternion and zero orientation error
    target_quat_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z)
                       .normalized();
    X0_[3] = X0_[4] = X0_[5] = 0.0;

    // Set target velocities (indices 6-11)
    X0_[6] = msg->twist.twist.linear.x;
    X0_[7] = msg->twist.twist.linear.y;
    X0_[8] = msg->twist.twist.linear.z;
    X0_[9] = msg->twist.twist.angular.x;
    X0_[10] = msg->twist.twist.angular.y;
    X0_[11] = msg->twist.twist.angular.z;
  }

  static void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    X_[0] = msg->pose.pose.position.x;
    X_[1] = msg->pose.pose.position.y;
    X_[2] = msg->pose.pose.position.z;

    Eigen::Quaterniond q_current(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_current.normalize();
    // Cache world-from-body rotation for kinematics integration
    Rwb_ = q_current.toRotationMatrix();

    // Use the target quaternion in the same hemisphere as current to avoid
    // jumps due to the q and -q equivalence.
    Eigen::Quaterniond q_target = target_quat_;
    if (q_current.dot(q_target) < 0.0) {
      q_target.coeffs() *= -1.0;
    }

    // Relative rotation from current to target
    Eigen::Quaterniond q_err = q_current.conjugate() * q_target;
    // Ensure shortest representation (positive scalar part) to prevent 180° flips
    if (q_err.w() < 0.0) {
      q_err.coeffs() *= -1.0;
    }
    Eigen::AngleAxisd aa(q_err);
    Eigen::Vector3d rot_vec = aa.axis() * aa.angle();
    X_[3] = rot_vec[0];
    X_[4] = rot_vec[1];
    X_[5] = rot_vec[2];

    X_[6] = msg->twist.twist.linear.x;
    X_[7] = msg->twist.twist.linear.y;
    X_[8] = msg->twist.twist.linear.z;
    X_[9] = msg->twist.twist.angular.x;
    X_[10] = msg->twist.twist.angular.y;
    X_[11] = msg->twist.twist.angular.z;
  }

  void computeLqr() {
    ct::optcon::LQR<STATE_DIM, CONTROL_DIM> lqr =
        ct::optcon::LQR<STATE_DIM, CONTROL_DIM>();
    // Update the kinematic coupling from body-frame velocity to world position
    // using the latest rotation estimate to avoid circular drift.
    A_mat.block<3, 3>(0, 6) = dt_ * Rwb_;
    // std::cout << "A_mat: " << A_mat << std::endl;
    // std::cout << "B_mat: " << B_mat << std::endl;
    // std::cout << "Q_mat: " << Q_mat << std::endl;
    // std::cout << "R_mat: " << R_mat << std::endl;
    lqr.compute(Q_mat, R_mat, A_mat, B_mat, K_);
  }

  bool setThrustZeroCallback(barracuda_control::SetThrustZero::Request &req,
                             barracuda_control::SetThrustZero::Response &res) {
    thrust_zero_enabled = req.enable_thrust_zero;
    res.success = true;

    if (thrust_zero_enabled) {
      res.message = "Thrust set to zero - all thrusters disabled";
      ROS_INFO("Thrust zero enabled - all thrusters set to zero");
    } else {
      res.message = "Normal LQR control resumed";
      ROS_INFO("Thrust zero disabled - normal LQR control resumed");
    }

    return true;
  }

private:
  void getRosParamVector(ros::NodeHandle &nh, const std::string &param_name,
                         Eigen::VectorXd &vector, int size) {
    std::vector<double> values;
    if (nh.getParam(param_name, values)) {
      if (values.size() == size) {
        vector = Eigen::Map<Eigen::VectorXd>(values.data(), size);
        ROS_INFO("Loaded vector %s from ROS parameters.", param_name.c_str());
      } else {
        ROS_ERROR("Incorrect size for %s. Expected %d but got %d elements.",
                  param_name.c_str(), size, (int)values.size());
        vector = Eigen::VectorXd::Zero(size);
      }
    } else {
      ROS_ERROR("Failed to load vector: %s", param_name.c_str());
      vector = Eigen::VectorXd::Zero(size);
    }
  }

  void getRosParamMatrix(ros::NodeHandle &nh, const std::string &param_name,
                         Eigen::Matrix3d &matrix, bool use_identity_default) {
    std::vector<double> values;
    if (nh.getParam(param_name, values)) {
      if (values.size() == 9) {
        matrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            values.data());
        ROS_INFO("Loaded matrix %s from ROS parameters.", param_name.c_str());
      } else {
        ROS_ERROR("Incorrect size for %s. Expected 9 but got %d elements.",
                  param_name.c_str(), (int)values.size());
        if (use_identity_default) {
          matrix.setIdentity();
        } else {
          matrix.setZero();
        }
      }
    } else {
      ROS_WARN("Failed to load matrix: %s, using default.", param_name.c_str());
      if (use_identity_default) {
        matrix.setIdentity();
      } else {
        matrix.setZero();
      }
    }
  }

  // Try to read a 6x6 matrix in row-major order from a flat vector parameter
  // with 36 elements. Returns true if the parameter exists and has correct
  // size; otherwise leaves 'matrix' unchanged and returns false.
  bool getRosParamMatrix6(ros::NodeHandle &nh, const std::string &param_name,
                          Eigen::Matrix<double, 6, 6> &matrix) {
    std::vector<double> values;
    if (!nh.getParam(param_name, values)) {
      return false;
    }
    if (values.size() != 36) {
      ROS_ERROR("Incorrect size for %s. Expected 36 but got %d elements.",
                param_name.c_str(), (int)values.size());
      return false;
    }
    matrix = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(values.data());
    ROS_INFO("Loaded 6x6 matrix %s from ROS parameters.", param_name.c_str());
    return true;
  }

  ros::NodeHandle &nh_;
  ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM> K_;
  Eigen::MatrixXd A_mat;
  Eigen::MatrixXd B_mat;
  Eigen::MatrixXd Q_mat;
  Eigen::MatrixXd R_mat;
  double smoothing_alpha_ = 1.0;
  Eigen::VectorXd U_prev_;
  static Eigen::Matrix3d Rwb_;
  ros::Publisher control_pub;
  ros::Subscriber odometry_sub;
  ros::Subscriber target_odometry_sub;
  ros::ServiceServer thrust_zero_service;
};

double LqrNode::X_[STATE_DIM] = {0};
double LqrNode::X0_[STATE_DIM] = {0};
Eigen::Quaterniond LqrNode::target_quat_(1, 0, 0, 0);
Eigen::Matrix3d LqrNode::Rwb_ = Eigen::Matrix3d::Identity();

int main(int argc, char **argv) {
  ros::init(argc, argv, "lqr_node");
  ros::NodeHandle nh;

  LqrNode lqr_node(nh);
  lqr_node.run();

  return 0;
}
