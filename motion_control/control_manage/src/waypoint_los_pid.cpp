#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <tf2/utils.h>
#include <iostream>

#include "los_guidance/set_of_los.h"
#include "follow_controller/pid_controller.h"
#include "control_manage/matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace motion_control;
ros::Publisher left_thrust_pub, right_thrust_pub, motion_path_pub;
nav_msgs::Path path;
nav_msgs::Path motion_path;
nav_msgs::Odometry odom;
ros::Timer control_cmd_pub;
bool is_final;

double desired_speed_test, desired_heading_test; //for turning pid params

//plot
vector<double> time_vec;
vector<double> speed_vec;
vector<double> target_speed_vec;
vector<double> course_vec;
vector<double> target_course_vec;
int step = 0;

unique_ptr<PidController> pid_ptr;
unique_ptr<SetOfLos> los_ptr;

void pathCallback(const nav_msgs::Path &msg) {
    path = msg;
}

void odomCallback(const nav_msgs::Odometry &msg) {
    odom = msg;
    
    motion_path.header.frame_id = "map";
    motion_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = path.header.stamp;
    pose_msg.header.frame_id = path.header.frame_id;
    pose_msg.pose.position.x = msg.pose.pose.position.x; 
    pose_msg.pose.position.y = msg.pose.pose.position.y; 
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.x = msg.pose.pose.orientation.x; 
    pose_msg.pose.orientation.y = msg.pose.pose.orientation.y; 
    pose_msg.pose.orientation.z = msg.pose.pose.orientation.z; 
    pose_msg.pose.orientation.w = msg.pose.pose.orientation.w; 
    motion_path.poses.push_back(pose_msg);
    motion_path_pub.publish(motion_path);

    
}

// double course2heading(double desired_course) {
//     Eigen::Vector2d vel_b;
//     pid_ptr->getVelInBodyFrame(vel_b, odom);
//     double beta_c = atan2(vel_b(1),sqrt(vel_b(1) * vel_b(1) + vel_b(0) * vel_b(0)));
//     //cout << "crab angle:" << beta_c << endl;
//     double desired_heading = desired_course - beta_c;

//     if (desired_heading > M_PI) {
//         desired_heading -= 2 * M_PI;
//     }
//     if (desired_heading < -M_PI) {
//         desired_heading += 2 * M_PI;
//     }
//     return desired_heading;
// }

void publish_control_cmd(const ros::TimerEvent &e) {
    if (path.poses.size() < 2) return;

    //los
    Eigen::Vector2d course_speed = los_ptr->getCourseFromWaypoints(path, odom, is_final);
    if (is_final) {
        cout << "stop" << endl;
        return;
    }
    double desired_course = course_speed(0);
    double desired_speed = course_speed(1);

    //pid course and speed control
    pid_ptr->setOdometry(odom); //do not forget
    // double tau_yaw_moment = pid_ptr->heading2YawMoment(desired_heading);
    double tau_surge_force = pid_ptr->speed2SurgeForce(desired_speed);
    //double tau_surge_force = 0.0;
    cout << "tau_surge_force:" << tau_surge_force << endl;    
    double tau_yaw_moment = pid_ptr->course2YawMoment(desired_course);
    cout << "tau_yaw_moment:" << tau_yaw_moment << endl;

    //thrust allocation and publish
    Eigen::Vector2d thrust = pid_ptr->thrustAllocation(tau_surge_force, tau_yaw_moment);
    std_msgs::Float32 left_thrust, right_thrust;
    left_thrust.data = thrust(0) / 250;
    right_thrust.data = thrust(1) / 250;
    cout << "left_thrust:" << thrust(0) << "right_thrust:" << thrust(1) << endl;
    left_thrust_pub.publish(left_thrust);
    right_thrust_pub.publish(right_thrust);

    //plot
    Eigen::Vector2d vel_b;
    pid_ptr->getVelInBodyFrame(vel_b, odom);
    time_vec.push_back(step);
    speed_vec.push_back(vel_b(0));
    target_speed_vec.push_back(desired_speed);

    course_vec.push_back(atan2(odom.twist.twist.linear.y, odom.twist.twist.linear.x));
    target_course_vec.push_back(desired_course);

    plt::subplot(2,1,1);
    plt::plot(time_vec, speed_vec);
    plt::plot(time_vec, target_speed_vec);
    plt::title("speed");

    plt::subplot(2,1,2);
    plt::plot(time_vec, course_vec);
    plt::plot(time_vec, target_course_vec);
    plt::title("course");
    plt::pause(0.001);
    ++step;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "los_pid_follow");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    
    left_thrust_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
    right_thrust_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/robot_path", 1);
    ros::Subscriber odom_sub = nh.subscribe("/wamv/sensors/position/p3d_wamv", 1, &odomCallback);
    ros::Subscriber path_sub = nh.subscribe("/waypoints_path", 1, &pathCallback);
    control_cmd_pub = nh.createTimer(ros::Duration(0.1), publish_control_cmd);
    pid_ptr.reset(new PidController);
    los_ptr.reset(new SetOfLos);
    pid_ptr->setParam(nh_p);
    los_ptr->setParam(nh_p);

    // nh_p.getParam("desired_speed", desired_speed_test);
    // nh_p.getParam("desired_heading", desired_heading_test);

    plt::ion(); //turn on interactive model

    ros::spin();

    plt::show();

    return 0;

}


