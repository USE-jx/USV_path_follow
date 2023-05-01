#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber goal_sub;
ros::Publisher waypoints_pub;
ros::Publisher point_marker_pub;

nav_msgs::Path waypoints_msg;
visualization_msgs::Marker marker;

void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
    
    waypoints_msg.header.frame_id = "map";
    waypoints_msg.header.stamp = ros::Time::now(); 
    waypoints_msg.poses.push_back(*goal);
    waypoints_pub.publish(waypoints_msg);

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
     marker.action = visualization_msgs::Marker::ADD;
    //marker.pose = goal->pose;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    geometry_msgs::Point pt;
    pt.x = goal->pose.position.x;
    pt.y = goal->pose.position.y;
    pt.z = goal->pose.position.z;

    marker.points.push_back(pt);
    point_marker_pub.publish(marker);

}
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "waypoints_publisher");
    ros::NodeHandle nh;

    goal_sub = nh.subscribe("move_base_simple/goal", 1, goalCallback);
    waypoints_pub = nh.advertise<nav_msgs::Path>("/waypoints_path", 10);
    point_marker_pub = nh.advertise<visualization_msgs::Marker>("/point", 10);

    ros::spin();
    return 0;

}
