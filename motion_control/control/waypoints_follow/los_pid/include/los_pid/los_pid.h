#ifndef LOS_PID_H
#define LOS_PID_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Eigen>
#include <cmath>
#include <tf2/utils.h>
#include "pid/pid_controller.h"
#include <iostream>
#include "matplotlibcpp.h"



namespace auto_recovery {

class LOS_PID {

private:

    //LOS相关参数
    //前视距离
    const double DELTA_ = 4;
    int index_ = 0;
    double switch_dist = 5;
    // //时变的前视距离
    // double delta_max_ = 12.0;
    // double delta_min_ = 1.0;
    // double delta_k_ = 1.0;
    // //circle of acceptance
    // double R_ = 1.0;
    // double max_speed_;
    // double max_speed_turn_;
    // double min_speed_;

    //期望速度朝向
    double desired_speed_ = 0.8, desired_heading_;
    //----------------------------------//
    //速度朝向PID相关参数
    PIDController speed_pid_;
    PIDController heading_pid_;
    double kp_psi_, ki_psi_, kd_psi_;
    double kp_u_, ki_u_, kd_u_;
    double dt_ = 0.01;
    double prev_error_ = 0.0;
    double error_ = 0;
    double last_error_ = 0;
    double i_term_ = 0;
    double d_term_ = 0;
    //ros related
    ros::NodeHandle nh_, nh_p_;
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    ros::Publisher left_thrust_pub_, right_thrust_pub_, path_pub_;
    ros::Subscriber path_sub_, odom_sub_;
    nav_msgs::Path path_msg_;
    ros::Timer timer_;
    ros::Time start_time_;
    bool haspath = false;
    std::vector<double> time_vec_;
    std::vector<double> speed_vec_;
    std::vector<double> target_speed_vec_;
    std::vector<double> speed_error_vec_;
    std::vector<double> heading_vec_;
    std::vector<double> target_heading_vec_;
    std::vector<double> heading_error_vec_;


    void odomCallback(const nav_msgs::Odometry &msg);
    void newPath(const nav_msgs::Path& path);
    void getDesiredSpeedHeading(double &desired_speed, double &desired_heading);
    void pubThrustCmd();
    double calculateSurgeForce(double dt, double desired_speed);
    double calculateYawMoment(double dt, double desired_heading);
    Eigen::Vector2d thrustAllocation(double tau_surge, double tau_yaw_moment);
    void timerCallback(const ros::TimerEvent &e);

public:
    LOS_PID(const ros::NodeHandle &nh, const ros::NodeHandle &nh_p);
    ~LOS_PID();

};

}

#endif