#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "barracuda_control/srv/set_thrust_zero.hpp"

class PIDNode : public rclcpp::Node
{
public:
    PIDNode()
        : Node("pid_node"), thrust_zero_enabled_(false)
    {
        this->declare_parameter("pid.update_rate", 50);
        rate_ = this->get_parameter("pid.update_rate").as_int();
        std::vector<double> kp_vals(6), ki_vals(6), kd_vals(6);
        getRosParamVector("pid.Kp", kp_vals);
        getRosParamVector("pid.Ki", ki_vals);
        getRosParamVector("pid.Kd", kd_vals);
        Kp_ = Eigen::Map<Eigen::Matrix<double,6,1>>(kp_vals.data()).asDiagonal();
        Ki_ = Eigen::Map<Eigen::Matrix<double,6,1>>(ki_vals.data()).asDiagonal();
        Kd_ = Eigen::Map<Eigen::Matrix<double,6,1>>(kd_vals.data()).asDiagonal();
        e_integral_.setZero();
        e_prev_.setZero();
        T_map_robot_.setIdentity();
        T_map_target_.setIdentity();
        last_time_ = this->now();
        control_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("thruster_manager/input", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered/local", 10,
            std::bind(&PIDNode::odometryCallback, this, std::placeholders::_1));
        target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "target_odometry", 10,
            std::bind(&PIDNode::targetCallback, this, std::placeholders::_1));
        thrust_zero_srv_ = this->create_service<barracuda_control::srv::SetThrustZero>(
            "set_thrust_zero",
            std::bind(&PIDNode::setThrustZeroCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate_),
            std::bind(&PIDNode::controlLoop, this));
    }

private:
    void controlLoop()
    {
        computeControl();
        publishControl();
    }

    void computeControl()
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        Eigen::Isometry3d T_error = T_map_robot_.inverse() * T_map_target_;
        Eigen::Vector3d linear_error = T_error.translation();
        Eigen::Quaterniond q_err(T_error.rotation());
        q_err.normalize();
        Eigen::AngleAxisd aa(q_err);
        Eigen::Vector3d angular_error = aa.angle() * aa.axis();
        Eigen::Matrix<double,6,1> e;
        e << linear_error, angular_error;
        // Log target pose and current pose
        Eigen::Quaterniond q_target(T_map_target_.rotation());
        q_target.normalize();
        Eigen::Quaterniond q_current(T_map_robot_.rotation());
        q_current.normalize();
        RCLCPP_INFO_STREAM(this->get_logger(),
            "PID target pose - pos: "
            << T_map_target_.translation().transpose()
            << ", quat: [" << q_target.w() << ", " << q_target.x() << ", "
            << q_target.y() << ", " << q_target.z() << "]");
        RCLCPP_INFO_STREAM(this->get_logger(),
            "PID current pose - pos: "
            << T_map_robot_.translation().transpose()
            << ", quat: [" << q_current.w() << ", " << q_current.x() << ", "
            << q_current.y() << ", " << q_current.z() << "]");
        if (dt > 0.0)
        {
            e_integral_ += e * dt;
            e_derivative_ = (e - e_prev_) / dt;
        }
        e_prev_ = e;
        control_ = Kp_ * e + Ki_ * e_integral_ + Kd_ * e_derivative_;
        RCLCPP_INFO_STREAM(this->get_logger(), "PID error: " << e.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "PID output: " << control_.transpose());
    }

    void publishControl()
    {
        auto msg = geometry_msgs::msg::Wrench();
        if (thrust_zero_enabled_)
        {
            msg.force.x = msg.force.y = msg.force.z = 0.0;
            msg.torque.x = msg.torque.y = msg.torque.z = 0.0;
        }
        else
        {
            msg.force.x = control_[0];
            msg.force.y = control_[1];
            msg.force.z = control_[2];
            msg.torque.x = control_[3];
            msg.torque.y = control_[4];
            msg.torque.z = control_[5];
        }
        control_pub_->publish(msg);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
        q.normalize();
        T_map_robot_.linear() = q.toRotationMatrix();
        T_map_robot_.translation() = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
    }

    void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
        q.normalize();
        T_map_target_.linear() = q.toRotationMatrix();
        T_map_target_.translation() = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
    }

    void setThrustZeroCallback(
        const std::shared_ptr<barracuda_control::srv::SetThrustZero::Request> request,
        std::shared_ptr<barracuda_control::srv::SetThrustZero::Response> response)
    {
        thrust_zero_enabled_ = request->enable_thrust_zero;
        response->success = true;
        response->message = thrust_zero_enabled_ ? "Thrust zero enabled" : "Thrust zero disabled";
    }

    void getRosParamVector(const std::string& name, std::vector<double>& vec)
    {
        this->declare_parameter(name, std::vector<double>(6, 0.0));
        vec = this->get_parameter(name).as_double_array();
        if (vec.size() != 6)
        {
            vec.resize(6, 0.0);
            RCLCPP_WARN(this->get_logger(), "%s should have 6 elements", name.c_str());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr control_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
    rclcpp::Service<barracuda_control::srv::SetThrustZero>::SharedPtr thrust_zero_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    int rate_;
    bool thrust_zero_enabled_;
    Eigen::Matrix<double,6,6> Kp_;
    Eigen::Matrix<double,6,6> Ki_;
    Eigen::Matrix<double,6,6> Kd_;
    Eigen::Matrix<double,6,1> e_integral_;
    Eigen::Matrix<double,6,1> e_derivative_;
    Eigen::Matrix<double,6,1> e_prev_;
    Eigen::Matrix<double,6,1> control_;
    Eigen::Isometry3d T_map_robot_;
    Eigen::Isometry3d T_map_target_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
