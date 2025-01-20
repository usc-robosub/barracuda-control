#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <ros/ros.h>

#define STATE_DIM 15
#define CONTROL_DIM 6

class LqrNode
{
public:
    static double X_[STATE_DIM];
    static double X0_[STATE_DIM];

    LqrNode(ros::NodeHandle& nh)
        : nh_(nh)
    {
        int rate;
        double dt;
        nh.getParam("lqr/publish_rate", rate);

        dt = 1.0 / rate;

        // Define the state and control matrices
        // Create an identity matrix of size 3x3 for linear and angular components
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);

        ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM> K_;
        Eigen::MatrixXd A_mat;
        Eigen::MatrixXd B_mat;
        Eigen::MatrixXd Q_mat;
        Eigen::MatrixXd R_mat;
        // Fill A_d based on the discrete-time formula
        A_mat.block(0, 0, 6, 6) = I6;                    // Position and orientation update
        A_mat.block(0, 6, 6, 6) = dt * I6;           // Velocity contribution
        A_mat.block(0, 12, 3, 3) = 0.5 * dt * dt * I3;  // Acceleration contribution (only linear)

        A_mat.block(6, 6, 6, 6) = I6;                     // Velocity update
        A_mat.block(6, 12, 3, 3) = dt * I3;           // Acceleration contribution (only linear)

        A_mat.block(12, 12, 3, 3) = I3;                   // Linear acceleration remains the same

    

        B_mat.block(0, 0, 3, 3) = 0.5 * dt * dt * I3;  // Position effect (linear)
        B_mat.block(6, 0, 3, 3) = dt * I3;                  // Velocity effect (linear)
        B_mat.block(12, 0, 3, 3) = I3;                           // Acceleration effect (linear)

        B_mat.block(0, 3, 3, 3) = 0.5 * dt * dt * I3;  // Orientation effect (rotational)
        B_mat.block(6, 3, 3, 3) = dt * I3;                  // Angular velocity effect


        Eigen::VectorXd Q_vector = Eigen::VectorXd(STATE_DIM);
        Eigen::VectorXd R_vector = Eigen::VectorXd(CONTROL_DIM);

        getRosParamVector(nh, "lqr/Q", Q_vector, STATE_DIM);
        getRosParamVector(nh, "lqr/R", R_vector, CONTROL_DIM);

        Q_mat = Q_vector.asDiagonal();
        R_mat = R_vector.asDiagonal();

        // Subscribe to the odometry and acceleration topics
        odometry_sub = nh.subscribe("odometry", 1, odometryCallback);
        acceleration_sub = nh.subscribe("acceleration", 1, accelerationCallback);

        // Publish the control input
        control_pub = nh.advertise<geometry_msgs::Wrench>("thruster_manager/input", 1);
    }

    void run()
    {
        ros::Rate rate(rate);
        while (ros::ok())
        {
            computeLqr();
            publishControl();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void publishControl()
    {
        Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd>(X_, STATE_DIM);
        Eigen::VectorXd X0 = Eigen::Map<Eigen::VectorXd>(X0_, STATE_DIM);
        Eigen::VectorXd U = -K_ * (X - X0);
        geometry_msgs::Wrench control_msg;
        control_msg.force.x = U[0];
        control_msg.force.y = U[1];
        control_msg.force.z = U[2];
        control_msg.torque.x = U[3];
        control_msg.torque.y = U[4];
        control_msg.torque.z = U[5];
        control_pub.publish(control_msg);
    }

    static void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        X_[0] = msg->pose.pose.position.x;
        X_[1] = msg->pose.pose.position.y;
        X_[2] = msg->pose.pose.position.z;
        
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        X_[3] = euler[0];
        X_[4] = euler[1];
        X_[5] = euler[2];

        X_[6] = msg->twist.twist.linear.x;
        X_[7] = msg->twist.twist.linear.y;
        X_[8] = msg->twist.twist.linear.z;
        X_[9] = msg->twist.twist.angular.x;
        X_[10] = msg->twist.twist.angular.y;
        X_[11] = msg->twist.twist.angular.z;

    }
    static void accelerationCallback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg)
    {
        X_[12] = msg->accel.accel.linear.x;
        X_[13] = msg->accel.accel.linear.y;
        X_[14] = msg->accel.accel.linear.z;
    }


private:
    void getRosParamVector(ros::NodeHandle& nh, const std::string& param_name, Eigen::VectorXd& vector, int size)
    {
        std::vector<double> values;
        if (nh.getParam(param_name, values))
        {
            if (values.size() == size)
            {
                vector = Eigen::Map<Eigen::VectorXd>(values.data(), size);
                ROS_INFO("Loaded vector %s from ROS parameters.", param_name.c_str());
            }
            else
            {
                ROS_ERROR("Incorrect size for %s. Expected %d but got %d elements.", param_name.c_str(), size, (int)values.size());
                vector = Eigen::VectorXd::Zero(size);
            }
        }
        else
        {
            ROS_ERROR("Failed to load vector: %s", param_name.c_str());
            vector = Eigen::VectorXd::Zero(size);
        }
    }

    void computeLqr()
    {
        ct::optcon::LQR<STATE_DIM, CONTROL_DIM> lqr;
        lqr.compute(Q_mat, R_mat, A_mat, B_mat, K_);
    }

    ros::NodeHandle& nh_;
    ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM> K_;
    Eigen::MatrixXd A_mat;
    Eigen::MatrixXd B_mat;
    Eigen::MatrixXd Q_mat;
    Eigen::MatrixXd R_mat;
    ros::Publisher control_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber acceleration_sub;
};

double LqrNode::X_[STATE_DIM] = {0};
double LqrNode::X0_[STATE_DIM] = {0};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lqr_node");
    ros::NodeHandle nh;

    LqrNode lqr_node(nh);

    
    return 0;
}