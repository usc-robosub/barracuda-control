#include "barracuda_control/srv/set_thrust_zero.hpp"
#include "barracuda_control/auv_lqr.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <string>

#define STATE_DIM 12
#define CONTROL_DIM 6

class LqrNode : public rclcpp::Node {
public:
  static double X_[STATE_DIM];
  static double X0_[STATE_DIM];
  static double q0_state_;
  static double q0_ref_;
  int rate;
  bool thrust_zero_enabled;

  LqrNode()
      : Node("lqr_node"), thrust_zero_enabled(false), tf_buffer_(this->get_clock()), 
        tf_listener_(tf_buffer_) {
    RCLCPP_INFO(this->get_logger(), "Discrete-time LQR Node initialized and running.");
    double dt;
    this->declare_parameter("lqr.update_rate", 50);
    rate = this->get_parameter("lqr.update_rate").as_int();
    dt = 1.0 / rate;

    // Vehicle physical parameters. Defaults are identity/zero but can be
    // overridden via ROS parameters.
    Eigen::Matrix3d M_t = Eigen::Matrix3d::Identity();   // translational mass
    Eigen::Matrix3d I_rot = Eigen::Matrix3d::Identity(); // rotational inertia
    Eigen::Matrix3d D_t = Eigen::Matrix3d::Zero(); // translational damping
    Eigen::Matrix3d D_r = Eigen::Matrix3d::Zero(); // rotational damping
    getRosParamMatrix("lqr.M_t", M_t, true);
    getRosParamMatrix("lqr.I_rot", I_rot, true);
    getRosParamMatrix("lqr.D_t", D_t, false);
    getRosParamMatrix("lqr.D_r", D_r, false);

    // Load the weighting matrices Q and R from ROS parameters.
    Eigen::VectorXd Q_vector = Eigen::VectorXd(STATE_DIM);
    Eigen::VectorXd R_vector = Eigen::VectorXd(CONTROL_DIM);
    getRosParamVector("lqr.Q", Q_vector, STATE_DIM);
    getRosParamVector("lqr.R", R_vector, CONTROL_DIM);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_mat = Q_vector.asDiagonal();
    Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R_mat =
        R_vector.asDiagonal();

    lqr_ = std::make_shared<barracuda_control::AUVLQR>(M_t, I_rot, D_t, D_r,
                                                       dt);
    lqr_->setCostMatrices(Q_mat, R_mat);

    // Resolve target body frame from params (supports optional tf_prefix)
    this->declare_parameter("thruster_manager.base_link", "barracuda_link");
    this->declare_parameter("thruster_manager.tf_prefix", "");
    base_link_frame_ = this->get_parameter("thruster_manager.base_link").as_string();
    std::string tf_prefix = this->get_parameter("thruster_manager.tf_prefix").as_string();
    
    if (!tf_prefix.empty()) {
      target_body_frame_ = tf_prefix + "/" + base_link_frame_;
    } else {
      target_body_frame_ = base_link_frame_;
    }

    // Set up ROS subscribers and publishers.
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered/local", 10,
        std::bind(&LqrNode::odometryCallback, this, std::placeholders::_1));
    
    target_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "target_odometry", 10,
        std::bind(&LqrNode::targetOdometryCallback, this, std::placeholders::_1));
    
    // Publish the control input
    control_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "thruster_manager/input", 10);

    // Set up service server for thrust zero control
    thrust_zero_service_ = this->create_service<barracuda_control::srv::SetThrustZero>(
        "set_thrust_zero",
        std::bind(&LqrNode::setThrustZeroCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Create timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / rate),
        std::bind(&LqrNode::controlLoop, this));
  }

  void controlLoop() {
    computeLqr();
    publishControl();
  }

  void publishControl() {
    auto control_msg = geometry_msgs::msg::Wrench();

    if (thrust_zero_enabled) {
      control_msg.force.x = 0.0;
      control_msg.force.y = 0.0;
      control_msg.force.z = 0.0;
      control_msg.torque.x = 0.0;
      control_msg.torque.y = 0.0;
      control_msg.torque.z = 0.0;
    } else {
      Eigen::Matrix<double, 12, 1> X =
          Eigen::Map<Eigen::Matrix<double, 12, 1>>(X_);
      Eigen::Matrix<double, 12, 1> X0 =
          Eigen::Map<Eigen::Matrix<double, 12, 1>>(X0_);
      // Log target pose, current pose, and error (position + orientation vector part)
      Eigen::Quaterniond q_state(q0_state_, X(3), X(4), X(5));
      Eigen::Quaterniond q_ref(q0_ref_, X0(3), X0(4), X0(5));
      q_state.normalize();
      q_ref.normalize();
      Eigen::Quaterniond q_error = q_ref * q_state.conjugate();
      // Enforce unique quaternion error representation (w >= 0) to avoid sign flips near pi
      if (q_error.w() < 0.0) q_error.coeffs() *= -1.0;
      RCLCPP_INFO_STREAM(this->get_logger(),
          "LQR target pose - pos: " << X0.head<3>().transpose() << ", quat: ["
                                     << q_ref.w() << ", " << q_ref.x() << ", "
                                     << q_ref.y() << ", " << q_ref.z() << "]");
      RCLCPP_INFO_STREAM(this->get_logger(),
          "LQR current pose - pos: " << X.head<3>().transpose() << ", quat: ["
                                     << q_state.w() << ", " << q_state.x()
                                     << ", " << q_state.y() << ", "
                                     << q_state.z() << "]");
      Eigen::Matrix<double, 3, 1> pos_err = X.head<3>() - X0.head<3>();
      Eigen::Matrix<double, 3, 1> att_err = -q_error.vec();
      RCLCPP_INFO_STREAM(this->get_logger(), "LQR error - pos: " << pos_err.transpose()
                                          << ", att_vec: "
                                          << att_err.transpose());
      Eigen::Matrix<double, 6, 1> U =
          lqr_->computeWrench(X, X0, q0_state_, q0_ref_);

      // The LQR outputs a wrench in the body frame. Publish directly as such.
      control_msg.force.x = U[0];
      control_msg.force.y = U[1];
      control_msg.force.z = U[2];
      control_msg.torque.x = U[3];
      control_msg.torque.y = U[4];
      control_msg.torque.z = U[5];
    }

    control_pub_->publish(control_msg);
  }

  void targetOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    X0_[0] = msg->pose.pose.position.x;
    X0_[1] = msg->pose.pose.position.y;
    X0_[2] = msg->pose.pose.position.z;

    Eigen::Quaterniond q_target(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_target.normalize();
    q0_ref_ = q_target.w();
    X0_[3] = q_target.x();
    X0_[4] = q_target.y();
    X0_[5] = q_target.z();

    X0_[6] = msg->twist.twist.linear.x;
    X0_[7] = msg->twist.twist.linear.y;
    X0_[8] = msg->twist.twist.linear.z;
    X0_[9] = msg->twist.twist.angular.x;
    X0_[10] = msg->twist.twist.angular.y;
    X0_[11] = msg->twist.twist.angular.z;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    X_[0] = msg->pose.pose.position.x;
    X_[1] = msg->pose.pose.position.y;
    X_[2] = msg->pose.pose.position.z;

    Eigen::Quaterniond q_current(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_current.normalize();
    q0_state_ = q_current.w();
    X_[3] = q_current.x();
    X_[4] = q_current.y();
    X_[5] = q_current.z();

    X_[6] = msg->twist.twist.linear.x;
    X_[7] = msg->twist.twist.linear.y;
    X_[8] = msg->twist.twist.linear.z;
    X_[9] = msg->twist.twist.angular.x;
    X_[10] = msg->twist.twist.angular.y;
    X_[11] = msg->twist.twist.angular.z;
  }

  void computeLqr() {
    // LQR gain computed internally by controller
  }

  void setThrustZeroCallback(
      const std::shared_ptr<barracuda_control::srv::SetThrustZero::Request> request,
      std::shared_ptr<barracuda_control::srv::SetThrustZero::Response> response) {
    thrust_zero_enabled = request->enable_thrust_zero;
    response->success = true;

    if (thrust_zero_enabled) {
      response->message = "Thrust set to zero - all thrusters disabled";
      RCLCPP_INFO(this->get_logger(), "Thrust zero enabled - all thrusters set to zero");
    } else {
      response->message = "Normal LQR control resumed";
      RCLCPP_INFO(this->get_logger(), "Thrust zero disabled - normal LQR control resumed");
    }
  }

private:
  void getRosParamVector(const std::string &param_name,
                         Eigen::VectorXd &vector, int size) {
    this->declare_parameter(param_name, std::vector<double>(size, 0.0));
    std::vector<double> values = this->get_parameter(param_name).as_double_array();
    
    if (values.size() == static_cast<size_t>(size)) {
      vector = Eigen::Map<Eigen::VectorXd>(values.data(), size);
      RCLCPP_INFO(this->get_logger(), "Loaded vector %s from ROS parameters.", param_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Incorrect size for %s. Expected %d but got %zu elements.",
                param_name.c_str(), size, values.size());
      vector = Eigen::VectorXd::Zero(size);
    }
  }

  void getRosParamMatrix(const std::string &param_name,
                         Eigen::Matrix3d &matrix, bool use_identity_default) {
    this->declare_parameter(param_name, std::vector<double>(9, use_identity_default ? 1.0 : 0.0));
    std::vector<double> values = this->get_parameter(param_name).as_double_array();
    
    if (values.size() == 9) {
      matrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          values.data());
      RCLCPP_INFO(this->get_logger(), "Loaded matrix %s from ROS parameters.", param_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Incorrect size for %s. Expected 9 but got %zu elements.",
                param_name.c_str(), values.size());
      if (use_identity_default) {
        matrix.setIdentity();
      } else {
        matrix.setZero();
      }
    }
  }

  std::shared_ptr<barracuda_control::AUVLQR> lqr_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr control_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odometry_sub_;
  rclcpp::Service<barracuda_control::srv::SetThrustZero>::SharedPtr thrust_zero_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_link_frame_;
  std::string target_body_frame_;
};

double LqrNode::X_[STATE_DIM] = {0};
double LqrNode::X0_[STATE_DIM] = {0};
double LqrNode::q0_state_ = 1.0;
double LqrNode::q0_ref_ = 1.0;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LqrNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
