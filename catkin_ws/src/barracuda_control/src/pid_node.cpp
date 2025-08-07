#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "barracuda_control/SetThrustZero.h"

class PIDNode
{
public:
    PIDNode(ros::NodeHandle& nh)
        : nh_(nh), thrust_zero_enabled_(false), last_time_(ros::Time::now())
    {
        nh_.getParam("pid/update_rate", rate_);
        std::vector<double> kp_vals(6), ki_vals(6), kd_vals(6);
        getRosParamVector(nh_, "pid/Kp", kp_vals);
        getRosParamVector(nh_, "pid/Ki", ki_vals);
        getRosParamVector(nh_, "pid/Kd", kd_vals);
        Kp_ = Eigen::Map<Eigen::Matrix<double,6,1>>(kp_vals.data()).asDiagonal();
        Ki_ = Eigen::Map<Eigen::Matrix<double,6,1>>(ki_vals.data()).asDiagonal();
        Kd_ = Eigen::Map<Eigen::Matrix<double,6,1>>(kd_vals.data()).asDiagonal();
        e_integral_.setZero();
        e_prev_.setZero();
        T_map_robot_.setIdentity();
        T_map_target_.setIdentity();
        control_pub_ = nh_.advertise<geometry_msgs::Wrench>("thruster_manager/input", 1);
        odom_sub_ = nh_.subscribe("odometry/filtered", 1, &PIDNode::odometryCallback, this);
        target_sub_ = nh_.subscribe("target_odometry", 1, &PIDNode::targetCallback, this);
        thrust_zero_srv_ = nh_.advertiseService("set_thrust_zero", &PIDNode::setThrustZeroCallback, this);
    }

    void run()
    {
        ros::Rate loop_rate(rate_);
        while (ros::ok())
        {
            computeControl();
            publishControl();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void computeControl()
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;
        Eigen::Isometry3d T_error = T_map_robot_.inverse() * T_map_target_;
        Eigen::Vector3d linear_error = T_error.translation();
        Eigen::Quaterniond q_err(T_error.rotation());
        q_err.normalize();
        Eigen::AngleAxisd aa(q_err);
        Eigen::Vector3d angular_error = aa.angle() * aa.axis();
        Eigen::Matrix<double,6,1> e;
        e << linear_error, angular_error;
        if (dt > 0.0)
        {
            e_integral_ += e * dt;
            e_derivative_ = (e - e_prev_) / dt;
        }
        e_prev_ = e;
        control_ = Kp_ * e + Ki_ * e_integral_ + Kd_ * e_derivative_;
    }

    void publishControl()
    {
        geometry_msgs::Wrench msg;
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
        control_pub_.publish(msg);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
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

    void targetCallback(const nav_msgs::Odometry::ConstPtr& msg)
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

    bool setThrustZeroCallback(barracuda_control::SetThrustZero::Request& req,
                               barracuda_control::SetThrustZero::Response& res)
    {
        thrust_zero_enabled_ = req.enable;
        res.success = true;
        return true;
    }

    void getRosParamVector(ros::NodeHandle& nh, const std::string& name, std::vector<double>& vec)
    {
        if (!nh.getParam(name, vec))
        {
            vec.assign(6, 0.0);
            ROS_WARN("Failed to load %s, using zeros", name.c_str());
        }
        else if (vec.size() != 6)
        {
            vec.resize(6, 0.0);
            ROS_WARN("%s should have 6 elements", name.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher control_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber target_sub_;
    ros::ServiceServer thrust_zero_srv_;
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
    ros::Time last_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle nh;
    PIDNode node(nh);
    node.run();
    return 0;
}

