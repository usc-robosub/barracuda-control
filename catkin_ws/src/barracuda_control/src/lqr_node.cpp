#include "barracuda_control/SetThrustZero.h"
#include "barracuda_control/auv_lqr.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vector>

#define STATE_DIM 12
#define CONTROL_DIM 6

class LqrNode {
public:
  static double X_[STATE_DIM];
  static double X0_[STATE_DIM];
  static double q0_state_;
  static double q0_ref_;
  int rate;
  bool thrust_zero_enabled;

  LqrNode(ros::NodeHandle &nh) : nh_(nh), thrust_zero_enabled(false) {
    ROS_INFO("Discrete-time LQR Node initialized and running.");
    double dt;
    nh.getParam("lqr/update_rate", rate);
    dt = 1.0 / rate;

    // Vehicle physical parameters. Defaults are identity/zero but can be
    // overridden via ROS parameters.
    Eigen::Matrix3d M_t = Eigen::Matrix3d::Identity();   // translational mass
    Eigen::Matrix3d I_rot = Eigen::Matrix3d::Identity(); // rotational inertia
    Eigen::Matrix3d D_t = Eigen::Matrix3d::Zero(); // translational damping
    Eigen::Matrix3d D_r = Eigen::Matrix3d::Zero(); // rotational damping
    getRosParamMatrix(nh, "lqr/M_t", M_t, true);
    getRosParamMatrix(nh, "lqr/I_rot", I_rot, true);
    getRosParamMatrix(nh, "lqr/D_t", D_t, false);
    getRosParamMatrix(nh, "lqr/D_r", D_r, false);

    // Load the weighting matrices Q and R from ROS parameters.
    Eigen::VectorXd Q_vector = Eigen::VectorXd(STATE_DIM);
    Eigen::VectorXd R_vector = Eigen::VectorXd(CONTROL_DIM);
    getRosParamVector(nh, "lqr/Q", Q_vector, STATE_DIM);
    getRosParamVector(nh, "lqr/R", R_vector, CONTROL_DIM);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_mat = Q_vector.asDiagonal();
    Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R_mat =
        R_vector.asDiagonal();

    lqr_ = std::make_shared<barracuda_control::AUVLQR>(M_t, I_rot, D_t, D_r,
                                                       dt);
    lqr_->setCostMatrices(Q_mat, R_mat);

    // Set up ROS subscribers and publishers.
    odometry_sub = nh.subscribe("odometry/filtered", 1, odometryCallback);
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
      Eigen::Matrix<double, 6, 1> U =
          lqr_->computeWrench(X, X0, q0_state_, q0_ref_);
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

  static void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
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
        matrix = use_identity_default ? Eigen::Matrix3d::Identity()
                                      : Eigen::Matrix3d::Zero();
      }
    } else {
      ROS_WARN("Failed to load matrix: %s, using default.", param_name.c_str());
      matrix = use_identity_default ? Eigen::Matrix3d::Identity()
                                    : Eigen::Matrix3d::Zero();
    }
  }

  ros::NodeHandle &nh_;
  std::shared_ptr<barracuda_control::AUVLQR> lqr_;
  ros::Publisher control_pub;
  ros::Subscriber odometry_sub;
  ros::Subscriber target_odometry_sub;
  ros::ServiceServer thrust_zero_service;
};

double LqrNode::X_[STATE_DIM] = {0};
double LqrNode::X0_[STATE_DIM] = {0};
double LqrNode::q0_state_ = 1.0;
double LqrNode::q0_ref_ = 1.0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lqr_node");
  ros::NodeHandle nh;

  LqrNode lqr_node(nh);
  lqr_node.run();

  return 0;
}
