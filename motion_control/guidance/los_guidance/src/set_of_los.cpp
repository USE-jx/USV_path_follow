#include "los_guidance/set_of_los.h"

namespace motion_control {

SetOfLos::SetOfLos() : index_(0), desired_course_(0), desired_speed_(0), is_final_(false) {}

SetOfLos::~SetOfLos() {}

void SetOfLos::setParam(ros::NodeHandle& nh) {
    nh.param("los/desired_speed", desired_speed_, 1.0);
    nh.param("los/switch_distance", switch_dist_, 7.0);
    nh.param("los/use_const_delta", use_const_delta_, true);
    nh.param("los/const_delta", const_delta_, 3.0);
    nh.param("los/delta_max", delta_max_, 10.0);
    nh.param("los/delta_min", delta_min_, 1.0);
    nh.param("los/gamma", gamma_, 1.0);
}

/**
 * @brief :get Course From Waypoints Path
 * 
 * @param path waypoints path
 * @param odom odometry
 * @param is_final if arrived final point
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d SetOfLos::getCourseFromWaypoints(const nav_msgs::Path &path, 
                                        const nav_msgs::Odometry &odom,
                                        bool &is_final) {
    if (path.poses.size() < 2) {
        return {tf2::getYaw(odom.pose.pose.orientation), 0};
    }

    //current position
    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;

    //calculate path-tangential angle
    double waypoint_cur_x = path.poses[index_].pose.position.x;
    double waypoint_cur_y = path.poses[index_].pose.position.y;

    double waypoint_next_x = path.poses[index_+1].pose.position.x;
    double waypoint_next_y = path.poses[index_+1].pose.position.y;
    double pi_p = atan2(waypoint_next_y - waypoint_cur_y, waypoint_next_x - waypoint_cur_x);

    //switch mechanism and final condition
    double dist = sqrt(pow(waypoint_next_x - current_x, 2) + pow(waypoint_next_y - current_y, 2));
    std::cout << "dist:" << dist << std::endl;
    if (dist  < switch_dist_) {
        std::cout << "go to next point" << std::endl;
        if (index_+1 == path.poses.size()-1) { //this is the last waypoint
            std::cout << "this is the last waypoint" << std::endl;
            if (dist  < 1) {
                std::cout << "stop" << std::endl;
                is_final = true;
                
                //cout << "current speed:" << vel_b(0) << endl;
            }
        } else {
            ++index_;
        }
    }

    //calculate along-track and cross track errors in p reference frame
    Eigen::Matrix2d R_p_n;
    Eigen::Vector2d track_errors_n;
    Eigen::Vector2d track_errors_p;
    R_p_n << cos(pi_p), -sin(pi_p), sin(pi_p), cos(pi_p);
    track_errors_n << current_x - waypoint_cur_x, current_y - waypoint_cur_y;
    track_errors_p = R_p_n.transpose() * track_errors_n;
    // cout << "track error x:" << track_errors_p(0) << endl;
    // cout << "track error y:" << track_errors_p(1) << endl;
   
    //calculate desired course,have two choices
    if (use_const_delta_) { //constant lookahead distance
        desired_course_ = pi_p - atan2(track_errors_p(1), const_delta_);
    } else { //time-varying looahead distance
        double delta_var = (delta_max_ - delta_min_) * std::exp(-gamma_ * std::pow(track_errors_p(1), 2)) + delta_min_;
        std::cout << "delta :" << delta_var << std::endl;
        //double delta_var = (delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(track_errors_p(1))) + delta_min_;
        desired_course_ = pi_p - atan2(track_errors_p(1), delta_var);
    }
    // cout << "desired course:" << desired_course << endl;
    
    return {desired_course_, desired_speed_};
}



}