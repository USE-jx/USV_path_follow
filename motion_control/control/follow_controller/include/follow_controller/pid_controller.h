#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf2/utils.h>

namespace motion_control {

struct params
{
    double kp_, ki_, kd_;
    double error_, prev_error_;
    double i_term_, d_term_;
    double max_output_, min_output_;
    double max_integral_;
    
};

class PidController {
private:
    params heading_params_;
    params course_params_;
    params speed_params_;
    nav_msgs::Odometry odom_;
    double dt_;

public:
    PidController();
    ~PidController();

    void setParam(ros::NodeHandle &nh);
    void setOdometry(nav_msgs::Odometry &odom);
    void getVelInBodyFrame(Eigen::Vector2d &vel_b, const nav_msgs::Odometry &odom);
    double heading2YawMoment(double desired_heading);
    double course2YawMoment(double desired_course);
    double speed2SurgeForce(double desired_speed);
    Eigen::Vector2d thrustAllocation(double tau_surge, double tau_yaw_moment);

};

}

#endif