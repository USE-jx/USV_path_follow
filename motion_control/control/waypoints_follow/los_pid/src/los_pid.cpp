#include "los_pid/los_pid.h"
using namespace std;

namespace plt = matplotlibcpp;

namespace auto_recovery
{

LOS_PID::LOS_PID(const ros::NodeHandle &nh, const ros::NodeHandle &nh_p) 
: nh_(nh), nh_p_(nh_p) {

    odom_sub_ = nh_.subscribe("/wamv/sensors/position/p3d_wamv", 1, &LOS_PID::odomCallback, this);
    path_sub_ = nh_.subscribe("/waypoints_path", 10, &LOS_PID::newPath, this);

    left_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);
    right_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/robot_path", 10);
    timer_ = nh_.createTimer(ros::Duration(0.1), &LOS_PID::timerCallback, this);
    //从参数服务器获取参数
    // //los
    // nh_p_.getParam("max_speed",max_speed_);
    // nh_p_.getParam("max_speed_turn",max_speed_turn_);
    // nh_p_.getParam("min_speed",min_speed_);
    //pid
    nh_p_.getParam("kp_psi",kp_psi_);
    nh_p_.getParam("ki_psi",ki_psi_);
    nh_p_.getParam("kd_psi",kd_psi_);
    nh_p_.getParam("kp_u",kp_u_);
    nh_p_.getParam("ki_u",ki_u_);
    nh_p_.getParam("kd_u",kd_u_);
    //nh_p_.getParam("desired_speed", desired_speed_);
    //nh_p_.getParam("desired_heading", desired_heading_);
    speed_pid_.kp_ = kp_u_;
    speed_pid_.ki_ = ki_u_;
    speed_pid_.kd_ = kd_u_;

    heading_pid_.kp_ = kp_psi_;
    heading_pid_.ki_ = ki_psi_;
    heading_pid_.kd_ = kd_psi_;
    plt::ion();
    start_time_ = ros::Time::now();
    
}
LOS_PID::~LOS_PID() {}

void LOS_PID::newPath(const nav_msgs::Path& path) {
    path_ = path;

}

void LOS_PID::odomCallback(const nav_msgs::Odometry &msg) {
    
    odom_ = msg;
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = path_msg_.header.stamp;
    pose_msg.header.frame_id = path_msg_.header.frame_id;
    pose_msg.pose.position.x = msg.pose.pose.position.x; 
    pose_msg.pose.position.y = msg.pose.pose.position.y; 
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.x = msg.pose.pose.orientation.x; 
    pose_msg.pose.orientation.y = msg.pose.pose.orientation.y; 
    pose_msg.pose.orientation.z = msg.pose.pose.orientation.z; 
    pose_msg.pose.orientation.w = msg.pose.pose.orientation.w; 
    path_msg_.poses.push_back(pose_msg);
    path_pub_.publish(path_msg_);
    
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    Eigen::Vector2d vel_b;
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y;
    vel_b = R_b_n.transpose() * vel_n;
        double current_speed = sqrt(pow(odom_.twist.twist.linear.x, 2) +
                                pow(odom_.twist.twist.linear.y, 2));
    double t = (ros::Time::now() - start_time_).toSec();
    double speed = vel_b(0) ;
    double heading = tf2::getYaw(odom_.pose.pose.orientation);
    double speed_error = desired_speed_ - speed;
    double heading_error = desired_heading_ - heading;
    time_vec_.push_back(t);
    speed_vec_.push_back(speed);
    heading_vec_.push_back(heading);
    target_speed_vec_.push_back(desired_speed_);
    speed_error_vec_.push_back(speed_error);
    target_heading_vec_.push_back(desired_heading_);
    heading_error_vec_.push_back(heading_error);

    plt::subplot(2, 1, 1);
    plt::plot(time_vec_, target_speed_vec_);
    plt::plot(time_vec_, speed_error_vec_);
    plt::plot(time_vec_, speed_vec_);
    plt::title("speed");
    plt::xlabel("t");
    //plt::xlim(0, 10);
    plt::ylim(0,2);
    
    plt::subplot(2, 1, 2);
    plt::plot(time_vec_, heading_vec_);
    plt::plot(time_vec_, target_heading_vec_);
    plt::plot(time_vec_, heading_error_vec_);

    plt::title("heading");
    plt::xlabel("t");
    //plt::xlim(0, 10);
    //plt::ylim(0,1);
    //plt::plot(time_vec_,heading_vec_);
    plt::pause(0.001);
    //plt::legend();

}
void LOS_PID::timerCallback(const ros::TimerEvent &e) {
    pubThrustCmd();


}

void LOS_PID::pubThrustCmd() {
     if (path_.poses.size() < 2) {
        return;
     }
     getDesiredSpeedHeading(desired_speed_, desired_heading_);
    cout << "desired_speed:" << desired_speed_ << endl;
    double tau_surge = calculateSurgeForce(0.01, desired_speed_);
    double tau_yaw_moment = calculateYawMoment(0.01, desired_heading_);
    //cout << "tau_surge:" << tau_surge << endl;
    //cout << "tau_yaw_moment:" << tau_yaw_moment << endl;

    Eigen::Vector2d thrust_cmd = thrustAllocation(tau_surge, tau_yaw_moment);
    cout << "thrust_cmd :" << thrust_cmd[0] << " " << thrust_cmd[1] << endl;
    std_msgs::Float32 left_thrust, right_thrust;
    // left_thrust.data = static_cast<float>(thrust_cmd[0]);
    // right_thrust.data = static_cast<float>(thrust_cmd[1]);
    left_thrust.data = thrust_cmd[0];
    right_thrust.data = thrust_cmd[1];
    //cout << "left_thrust" << left_thrust.data << endl;

    left_thrust_pub_.publish(left_thrust);
    right_thrust_pub_.publish(right_thrust);

}

void LOS_PID::getDesiredSpeedHeading(double &desired_speed, double &desired_heading) {
    if (path_.poses.size() < 2) 
    return;
    cout << "path size:" << path_.poses.size() << endl;
    double x = odom_.pose.pose.position.x;
    double y = odom_.pose.pose.position.y;
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    //cout << "current pose:" << x << y << psi << endl;

    
    //calculate path tangential angle 
    
    double x_next = path_.poses[index_ + 1].pose.position.x;
    double y_next = path_.poses[index_ + 1].pose.position.y;
    double x_cur = path_.poses[index_].pose.position.x;
    double y_cur = path_.poses[index_].pose.position.y;
    double pi_p = atan2(y_next - y_cur, x_next - x_cur);
    cout << "index:" << index_ << endl;
    cout << "x_cur:" << x_cur << endl;
    cout << "x_next:" << x_next << endl;
    cout << "path tangential angle:" << pi_p << endl;

    //calculate along-track and cross track errors in p
    Eigen::Matrix2d R_p_n;
    Eigen::Vector2d track_errors_n;
    Eigen::Vector2d track_errors_p;
    R_p_n << cos(pi_p), -sin(pi_p), sin(pi_p), cos(pi_p);
    track_errors_n << x - x_cur, y - y_cur;
    track_errors_p = R_p_n.transpose() * track_errors_n;
    cout << "track error x:" << track_errors_p(0) << endl;
    cout << "track error y:" << track_errors_p(1) << endl;

    //calculate desired course
    double desired_course = pi_p - atan2(track_errors_p(1), DELTA_);
    cout << "desired course:" << desired_course << endl;
    //desired_heading = desired_course;
    //calculate desired heading
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    Eigen::Vector2d vel_b;
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y;
    vel_b = R_b_n.transpose() * vel_n;
    double beta_c = atan2(vel_b(1),sqrt(vel_b(1) * vel_b(1) + vel_b(0) * vel_b(0)));
    cout << "sideslip angle:" << beta_c << endl;
    desired_heading = desired_course - beta_c;

    if (desired_heading > M_PI) {
        desired_heading -= 2 * M_PI;
    }
    if (desired_heading < -M_PI) {
        desired_heading += 2 * M_PI;
    }
    cout << "desired_heading:" << desired_heading << endl;
    cout << "current heading:" << psi << endl;
    //if approach next waypoint
    double dist = sqrt(pow(x_next - x, 2) + pow(y_next - y, 2));
    cout << "dist:" << dist << endl;
    if (dist  < switch_dist) {
        cout << "go to next point" << endl;
        if (index_+1 == path_.poses.size()-1) { //this is the last waypoint
            cout << "this is the last waypoint" << endl;
            if (dist  < 1) {
                cout << "stop" << endl;
                desired_heading = psi;
                desired_speed = 0;
                return;
                //cout << "current speed:" << vel_b(0) << endl;
            }
        } else {
            ++index_;
        }
    }
    //cout << "current speed:" << vel_b(0) << endl;


    // //如果执行完路径就停止保持朝向
    // if (path_.poses.size() <= 10) {
    //     desired_speed = 0.0;
    //     desired_heading = psi;
    //     cout << "stop" << endl;
    //     return;
    // }
    // //变视距LOS
    // //找到路径上距离最近的点
    // std::vector<geometry_msgs::PoseStamped>::iterator closest_it;
    // double min_dist = std::numeric_limits<double>::max();
    // for (auto it = path_.poses.begin(); it != path_.poses.end(); ++it) {
    //     double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
    //                             std::pow(y - it->pose.position.y, 2));
    //     if (dist < min_dist) {
    //         min_dist = dist;
    //         closest_it = it;
    //     }
    // }

    // geometry_msgs::PoseStamped pose_closest = * closest_it;
    // cout <<"最近点x:" << pose_closest.pose.position.x << endl;
    // cout <<"最近点y:" << pose_closest.pose.position.y << endl;

    // //删除路径上路径点之前的元素
    // path_.poses.erase(path_.poses.begin(), closest_it);
    // cout << "path pose size:" << path_.poses.size() << endl;

    // //计算最近点上路径的正切角
    // double gamma_p = tf2::getYaw(pose_closest.pose.orientation);
    // cout << "gamma_p:" << gamma_p << endl;

    // //计算cross track error
    // double y_e = - (x - pose_closest.pose.position.x) * std::sin(gamma_p) +
    //              (y - pose_closest.pose.position.y) * std::cos(gamma_p);
    // cout << "cross track error:" << endl;
    // //计算变化的前视距离
    // double delta_y_e = (delta_max_ - delta_min_) * std::exp(-delta_k_ * std::pow(y_e, 2)) + delta_min_;
    // cout << "前视距离:" << delta_y_e << endl;

    // //如果转向就设为最小的前视距离
    // bool is_turning = false;
    // if ((closest_it + 1) != path_.poses.end()) {
    //     double next_angle = tf2::getYaw((*(closest_it + 1)).pose.orientation);
    //     if (std::fabs(gamma_p - next_angle) > std::numeric_limits<double>::epsilon()) {
    //         delta_y_e = delta_min_;
    //         is_turning = true;
    //     }
    // } 

    // //得到期望朝向
    // //desired_heading = gamma_p + std::atan2(-y_e, delta_y_e);
    // desired_heading = std::atan2(-y_e, delta_y_e);
    // //desired_heading = -3;

    // //计算朝向误差
    // double heading_err = desired_heading - psi;
    // while (heading_err > M_PI) {
    //     heading_err -= 2 * M_PI;
    // }
    // while (heading_err < -M_PI) {
    //     heading_err += 2 * M_PI;
    // }
    // cout <<"desired heading:" << desired_heading << endl;
    // cout << "current heading:" << psi << endl;
    // //计算期望速度
    // //desired_speed = 0.3;
    //  desired_speed = max_speed_ * (1 - std::abs(y_e) / 5 - std::abs(heading_err) / M_PI_2);
    //  desired_speed = std::max(desired_speed, min_speed_);
    //  if (is_turning) desired_speed = max_speed_turn_;
    //   cout <<"desired speed:" << desired_speed << endl;
    //   cout <<"current speed:" << odom_.twist.twist.linear.x << endl;


}


double LOS_PID::calculateSurgeForce(double dt, double desired_speed) {
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    Eigen::Vector2d vel_b;
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y;
    vel_b = R_b_n.transpose() * vel_n;
    error_ = desired_speed - vel_b(0);
    // double current_speed = sqrt(pow(odom_.twist.twist.linear.x, 2) +
    //                             pow(odom_.twist.twist.linear.y, 2));
    // error_ = desired_speed - current_speed;
    cout << "error:" << error_ << endl;
    //if (error_ < 0.1 && error_ > -0.1) error_ = 0;

    double p_trem = kp_u_ * error_;

    i_term_ += ki_u_ * error_ * dt_;
    if (i_term_ > 1) {
        i_term_ = 1;
    }
    if (i_term_ < -1) {
        i_term_ = -1;
    }
    cout << "iterm:" << i_term_ << endl;
    d_term_ = kd_u_ * (error_ - last_error_) / dt_;

    double output = p_trem + i_term_ + d_term_;

    last_error_ = error_;

    return output;
    
}
double LOS_PID::calculateYawMoment(double dt, double desired_heading) {
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    //cout << "current_heading:" << psi << endl;
    // double current_course = atan2(odom_.twist.twist.linear.y, odom_.twist.twist.linear.x);
    heading_pid_.setCurrentState(psi);
    heading_pid_.setDesiredState(desired_heading);

    double output = heading_pid_.computeControlCmd();

    return output;
}
Eigen::Vector2d LOS_PID::thrustAllocation(double tau_surge, double tau_yaw_moment) {
    Eigen::Matrix2d T;
    T << 1, 1, -1.02, 1.02;
    //T << 100, 100, -102, 102;
    Eigen::Vector2d tau;
    tau[0] = tau_surge;
    tau[1] = tau_yaw_moment;
    Eigen::Vector2d thruster;
    thruster = T.inverse() * tau;
    return thruster;
}


} // namespace auto_recovery

