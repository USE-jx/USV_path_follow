#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry &msg) {

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //时间戳和父子坐标系
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "wamv/base_link";

    //旋转
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
    //平移
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;

    br.sendTransform(transformStamped);


}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "pub_tf_map2base");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/wamv/sensors/position/p3d_wamv", 10, &odomCallback);

    ros::spin();
    return 0;
}