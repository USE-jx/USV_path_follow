#include "follow_controller/pid_controller.h"

namespace motion_control {

PidController::PidController() {
    dt_ = 0.1;
    heading_params_.i_term_ = 0;
    heading_params_.d_term_ = 0;
    heading_params_.max_integral_ = std::numeric_limits<double>::max();
    heading_params_.max_output_ = std::numeric_limits<double>::max();
    heading_params_.min_output_ = -std::numeric_limits<double>::max();
    heading_params_.error_ = 0.0;
    heading_params_.prev_error_ = 0.0;

    course_params_.i_term_ = 0;
    course_params_.d_term_ = 0;
    course_params_.max_integral_ = std::numeric_limits<double>::max();
    course_params_.max_output_ = std::numeric_limits<double>::max();
    course_params_.min_output_ = -std::numeric_limits<double>::max();
    course_params_.error_ = 0.0;
    course_params_.prev_error_ = 0.0;

    speed_params_.i_term_ = 0;
    speed_params_.d_term_ = 0;
    speed_params_.max_integral_ = std::numeric_limits<double>::max();
    speed_params_.max_output_ = std::numeric_limits<double>::max();
    speed_params_.min_output_ = -std::numeric_limits<double>::max();
    speed_params_.error_ = 0.0;
    speed_params_.prev_error_ = 0.0;
}
PidController::~PidController() {}


void PidController::setParam(ros::NodeHandle &nh) {
    nh.param("pid/heading_kp", heading_params_.kp_, 3.0);
    nh.param("pid/heading_ki", heading_params_.ki_, 0.0);
    nh.param("pid/heading_kd", heading_params_.kd_, 0.0);

    nh.param("pid/course_kp", course_params_.kp_, 3.0);
    nh.param("pid/course_ki", course_params_.ki_, 0.0);
    nh.param("pid/course_kd", course_params_.kd_, 2.0);
    
    nh.param("pid/speed_kp", speed_params_.kp_, 3.0);
    nh.param("pid/speed_ki", speed_params_.ki_, 0.0);
    nh.param("pid/speed_kd", speed_params_.kd_, 2.0);

}
void PidController::setOdometry(nav_msgs::Odometry &odom) {
    odom_ = odom;
}

//no use 
double PidController::heading2YawMoment(double desired_heading) {

    double current_heading = tf2::getYaw(odom_.pose.pose.orientation);
    heading_params_.error_ = desired_heading - current_heading;
    std::cout << "heading error:" << heading_params_.error_ << std::endl;

    if (heading_params_.error_ > M_PI) {
        heading_params_.error_ -= 2 * M_PI;
    }
    if (heading_params_.error_ < -M_PI) {
        heading_params_.error_ += 2 * M_PI;
    } 
    //  if (heading_params_.error_ < 0.1 && heading_params_.error_ > -0.1) {
    //      heading_params_.error_ = 0;
    //  }

    double p_term = heading_params_.kp_ * heading_params_.error_;
    heading_params_.i_term_ += heading_params_.ki_ * heading_params_.error_ * dt_;

    if (heading_params_.i_term_ > heading_params_.max_integral_) {
        heading_params_.i_term_ = heading_params_.max_integral_;
    }
    if (heading_params_.i_term_ < -heading_params_.max_integral_) {
        heading_params_.i_term_ = -heading_params_.max_integral_;
    }

    heading_params_.d_term_ = heading_params_.kd_ * (heading_params_.error_ - heading_params_.prev_error_) / dt_;

    double output = p_term + heading_params_.i_term_ + heading_params_.d_term_;


    heading_params_.prev_error_ = heading_params_.error_;

    return output * 250;

}
// course is better than heading
double PidController::course2YawMoment(double desired_course) {

    double current_course = atan2(odom_.twist.twist.linear.y, odom_.twist.twist.linear.x);
    course_params_.error_ = desired_course - current_course;

    if (course_params_.error_ > M_PI) {
        course_params_.error_ -= 2 * M_PI;
    }
    if (course_params_.error_ < -M_PI) {
        course_params_.error_ += 2 * M_PI;
    } 

    double p_term = course_params_.kp_ * course_params_.error_;
    course_params_.i_term_ += course_params_.ki_ * course_params_.error_ * dt_;

    if (course_params_.i_term_ > course_params_.max_integral_) {
        course_params_.i_term_ = course_params_.max_integral_;
    }
    if (course_params_.i_term_ < -course_params_.max_integral_) {
        course_params_.i_term_ = -course_params_.max_integral_;
    }

    
    course_params_.d_term_ = course_params_.kd_ * (course_params_.error_ - course_params_.prev_error_) / dt_;
    // if (-0.01 < course_params_.error_ && course_params_.error_< 0.01) 
    // course_params_.d_term_ = 0;

    double output = 250 * (p_term + course_params_.i_term_ + course_params_.d_term_);
    std::cout << "coursemoment:" << output << std::endl;
    std::cout << "pterm:" << p_term << std::endl;
    std::cout << "iterm:" << course_params_.i_term_ << std::endl;
    std::cout << "dterm:" <<  course_params_.d_term_ << std::endl;
    std::cout << "error:" <<  course_params_.error_ << std::endl;
    std::cout << "error rate:" <<  (course_params_.error_ - course_params_.prev_error_) / dt_ << std::endl;
    

    // if (output > course_params_.max_output_) {
    //     output = course_params_.max_output_;
    // }
    // if (output < course_params_.min_output_) {
    //     output = course_params_.min_output_;
    // }

    course_params_.prev_error_ = course_params_.error_;

    return output;
}

//get velocity in body frame
void PidController::getVelInBodyFrame(Eigen::Vector2d &vel_b, const nav_msgs::Odometry &odom) {
    double psi = tf2::getYaw(odom.pose.pose.orientation);
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << odom.twist.twist.linear.x, odom.twist.twist.linear.y;
    vel_b = R_b_n.transpose() * vel_n;

}

double PidController::speed2SurgeForce(double desired_speed) {
    Eigen::Vector2d vel_b;
    getVelInBodyFrame(vel_b, odom_);
    double current_speed = vel_b(0);
    speed_params_.error_ = desired_speed - current_speed;
    // if (speed_params_.error_ < 0.05 && speed_params_.error_ > -0.05) {
    //     speed_params_.error_ = 0;
    // }
    double p_term = speed_params_.kp_ * speed_params_.error_;

    speed_params_.i_term_ += speed_params_.ki_ * speed_params_.error_ * dt_;

    if (speed_params_.i_term_ > speed_params_.max_integral_) {
        speed_params_.i_term_ = speed_params_.max_integral_;
    }
    if (speed_params_.i_term_ < -speed_params_.max_integral_) {
        speed_params_.i_term_ = -speed_params_.max_integral_;
    }

    speed_params_.d_term_ = speed_params_.kd_ * (speed_params_.error_ - speed_params_.prev_error_) / dt_;

    double output = (51.3+72.4*std::abs(desired_speed))*desired_speed + 250 * (p_term + speed_params_.i_term_ + speed_params_.d_term_);
    std::cout << "damp force: " << (51.3+72.4*abs(desired_speed))*desired_speed << std::endl;

    speed_params_.prev_error_ = speed_params_.error_;

    return output;
}

Eigen::Vector2d PidController::thrustAllocation(double tau_surge, double tau_yaw_moment) {
    Eigen::Matrix2d T;
    T << 1, 1, -1.02, 1.02; //1.02是螺旋桨和船中心的横向距离
    
    Eigen::Vector2d tau;
    tau[0] = tau_surge;
    tau[1] = tau_yaw_moment;
    Eigen::Vector2d thruster;
    thruster = T.inverse() * tau;
    return thruster;
}

}