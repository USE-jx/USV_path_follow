#include "los_pid/los_pid.h"
namespace plt = matplotlibcpp;
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "los_pid_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    auto_recovery::LOS_PID los_pid(nh, nh_p);

    ros::spin();
    plt::show();
    return 0;
}