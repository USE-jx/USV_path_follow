#ifndef SET_OF_LOS_H
#define SET_OF_LOS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf2/utils.h>

namespace motion_control {

class SetOfLos {
private:
    double desired_course_;
    double desired_speed_;
    bool is_final_;
    int index_;  //index of waypoints
    double switch_dist_;

    //if use const lookahead distance
    bool use_const_delta_;
    double const_delta_;

    //time-varying params
    double delta_max_;
    double delta_min_;
    double gamma_;

public:

    SetOfLos();
    ~SetOfLos();

    void setParam(ros::NodeHandle& nh);

    //get desired course from waypoints path
    Eigen::Vector2d getCourseFromWaypoints(const nav_msgs::Path &path, 
                                  const nav_msgs::Odometry &odom,
                                  bool &is_final);

};

}


#endif