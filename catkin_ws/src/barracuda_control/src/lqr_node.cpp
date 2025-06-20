#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define STATE_DIM 15
#define CONTROL_DIM 6

class LqrNode
{
public:
    static double X_[STATE_DIM];
    static double X0_[STATE_DIM];
    int rate;

    LqrNode(ros::NodeHandle& nh)
        : nh_(nh)
    {
        ROS_INFO("Discrete-time LQR Node initialized and running.");
        double dt;
        nh.getParam("lqr/update_rate", rate);
        dt = 1.0 / rate;

        // Define basic identity matrices for 3 and 15 dimensions.
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd I15 = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

        // Define your vehicle's physical parameters here.
        // These are placeholders – replace with your actual values.
        // Effective translational mass: rigid-body mass plus added mass.
        Eigen::MatrixXd M_t = Eigen::MatrixXd::Identity(3, 3);  
        // Effective rotational inertia: rigid-body inertia plus added inertia.
        Eigen::MatrixXd I_rot = Eigen::MatrixXd::Identity(3, 3);  
        // Damping matrices evaluated at the operating point.
        Eigen::MatrixXd D_t = Eigen::MatrixXd::Zero(3, 3);  // Translational damping
        Eigen::MatrixXd D_r = Eigen::MatrixXd::Zero(3, 3);  // Rotational damping

        // Precompute inverses (assumed invertible)
        Eigen::MatrixXd M_t_inv = M_t.inverse();
        Eigen::MatrixXd I_rot_inv = I_rot.inverse();

        // Build discrete-time state-space matrices directly.
        // State ordering: [ p (indices 0-2); theta (3-5); v (6-8); omega (9-11); a (12-14) ]
        Eigen::MatrixXd A_d = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(STATE_DIM, CONTROL_DIM);

        // Position update: p[k+1] = p[k] + dt * v[k]
        A_d.block(0, 0, 3, 3) = I3;
        A_d.block(0, 6, 3, 3) = dt * I3;

        // Orientation update: theta[k+1] = theta[k] + dt * omega[k]
        A_d.block(3, 3, 3, 3) = I3;
        A_d.block(3, 9, 3, 3) = dt * I3;

        // Velocity update: v[k+1] = v[k] + dt * a[k]
        A_d.block(6, 6, 3, 3) = I3;
        A_d.block(6, 12, 3, 3) = dt * I3;

        // Angular velocity update:
        // omega[k+1] = (I - dt*I_rot_inv*D_r) * omega[k] + dt*I_rot_inv*delta_M
        A_d.block(9, 9, 3, 3) = I3 - dt * I_rot_inv * D_r;

        // Acceleration update:
        // a[k+1] = (I - dt*M_t_inv*D_t) * a[k] + dt*M_t_inv*delta_F
        A_d.block(12, 12, 3, 3) = I3 - dt * M_t_inv * D_t;

        // Input matrix B_d.
        // Force input delta_F affects the acceleration dynamics.
        B_d.block(12, 0, 3, 3) = dt * M_t_inv;
        // Moment input delta_M affects the angular velocity dynamics.
        B_d.block(9, 3, 3, 3) = dt * I_rot_inv;

        ROS_INFO_STREAM("Discrete-time A matrix:\n" << A_d);
        ROS_INFO_STREAM("Discrete-time B matrix:\n" << B_d);

        // Load the weighting matrices Q and R from ROS parameters.
        Eigen::VectorXd Q_vector = Eigen::VectorXd(STATE_DIM);
        Eigen::VectorXd R_vector = Eigen::VectorXd(CONTROL_DIM);
        getRosParamVector(nh, "lqr/Q", Q_vector, STATE_DIM);
        getRosParamVector(nh, "lqr/R", R_vector, CONTROL_DIM);
        Eigen::MatrixXd Q_mat = Q_vector.asDiagonal();
        Eigen::MatrixXd R_mat = R_vector.asDiagonal();

        // Compute the discrete-time LQR gain using your chosen method.
        K_ = ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero();
        // [Insert your discrete-time LQR synthesis here using A_d, B_d, Q_mat, and R_mat.]

        // Set up ROS subscribers and publishers.
        odometry_sub = nh.subscribe("odometry/filtered", 1, odometryCallback);
        acceleration_sub = nh.subscribe("accel/filtered", 1, accelerationCallback);
        cmd_vel_sub = nh.subscribe("cmd_vel", 1, setpointCallback);
        // Publish the control input
        control_pub = nh.advertise<geometry_msgs::Wrench>("thruster_manager/input", 1);
    }

    void run() // main loop
    {
        ros::Rate rate(rate); //
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

    static void setpointCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        X0_[6] = msg->linear.x;
        X0_[7] = msg->linear.y;
        X0_[8] = msg->linear.z;
        X0_[9] = msg->angular.x;
        X0_[10] = msg->angular.y;
        X0_[11] = msg->angular.z;
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

    void computeLqr()
    {
        ct::optcon::LQR<STATE_DIM, CONTROL_DIM> lqr = ct::optcon::LQR<STATE_DIM, CONTROL_DIM>();
        // std::cout << "A_mat: " << A_mat << std::endl;
        // std::cout << "B_mat: " << B_mat << std::endl;
        // std::cout << "Q_mat: " << Q_mat << std::endl;
        // std::cout << "R_mat: " << R_mat << std::endl;
        lqr.compute(Q_mat, R_mat, A_mat, B_mat, K_);
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

    ros::NodeHandle& nh_;
    ct::core::FeedbackMatrix<STATE_DIM, CONTROL_DIM> K_;
    Eigen::MatrixXd A_mat;
    Eigen::MatrixXd B_mat;
    Eigen::MatrixXd Q_mat;
    Eigen::MatrixXd R_mat;
    ros::Publisher control_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber acceleration_sub;
    ros::Subscriber cmd_vel_sub;
};

double LqrNode::X_[STATE_DIM] = {0};
double LqrNode::X0_[STATE_DIM] = {0};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lqr_node");
    ros::NodeHandle nh;

    LqrNode lqr_node(nh);
    lqr_node.run();

    
    return 0;
}
