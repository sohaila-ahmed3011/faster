#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
#include "mpc_tracking_controller.h"

class MPCControl {
public:
    MPCControl(double delta_t, double min_error) : r_(0.3), L_(1.25), D_(0.07), Ts_(delta_t), t_(Eigen::ArrayXd::LinSpaced(10/Ts_, 0, 10)), end_controller_(false), error_(0.0), success_(false), min_acceptable_error_(min_error) {
        std::cout << "Initializing the MPC controller" << std::endl;
        pub_velocity_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        sub_odom_ = nh_.subscribe("/odometry/filtered", 10, &MPCControl::set_pose, this);
        i_ = 0;
    }

    void send_vel(double v, double w) {
        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        pub_velocity_.publish(msg);
    }

    void euler_from_quaternion(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
        tf::Quaternion tfq(q.x, q.y, q.z, q.w);
        tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
    }

    double wrap_to_pi(double x) {
        double xwrap = std::remainder(x, 2*M_PI);
        if (std::abs(xwrap) > M_PI) {
            xwrap -= 2*M_PI * std::signbit(xwrap);
        }
        return xwrap;
    }

    void set_pose(const nav_msgs::Odometry::ConstPtr& msg) {
        double roll, pitch, yaw;
        euler_from_quaternion(msg->pose.pose.orientation, roll, pitch, yaw);
        if (set_q_init_.size() == 0) {
            set_q_init_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
            q_ = set_q_init_;
        }
        odom_pose_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    }

    void mpc_planner_init(const Eigen::Vector3d& target_pose) {
        mpc_solver_ = new MPCDiffDriveControl(Ts_, 20, 1.0);
        target_pose_ = target_pose;
        if (q_.size() == 0) {
            std::cout << "Still robot current pose is not set" << std::endl;
        } else {
            mpc_solver_->init_regulator(q_, target_pose_);
        }
    }

    void move_one_step() {
        if (mpc_solver_->init_reg) {
            Eigen::VectorXd u = mpc_solver_->update(odom_pose_);
            double v = u(0);
            double w = u(1);
            send_vel(v, w);
        } else if (q_.size() != 0) {
            mpc_solver_->init_regulator(q_, target_pose_);
        }

        if (odom_pose_.size() != 0) {
            error_ = (odom_pose_ - target_pose_).norm();
            std::cout << "Error: " << error_



    // MPCControl(double delta_t, double min_error)
    // {
    //     std::cout << "Initializing the MPC controller" << std::endl;
    //     pub_velocity = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
    //     sub_odometry = nh.subscribe("/odometry/filtered", 10, &MPCControl::set_pose, this);
    //     i = 0;
    //     q_init_set = false;
    //     r = 0.3; // Wheel radius
    //     L = 1.25; // Axle length
    //     D = 0.07; // Distance between the front front whell and rear axle
    //     Ts = delta_t; // Sampling time
    //     t = casadi::DM::linspace(0, 10, 10/Ts + 1); // Simulation time
    //     end_controller = false;
    //     odom_pose = casadi::DM::zeros(3);
    //     error = 0;
    //     success = false;
    //     min_acceptable_error = min_error;
    // }