#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

std::string pseudo_child_frame_name;
std::string pseudo_parent_frame_name;

void poseCallBack(const nav_msgs::Odometry::ConstPtr& lidar_pose_msg)
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped lidar_to_local_origin;
    

    // Set msg fields value
    lidar_to_local_origin.header.stamp = lidar_pose_msg->header.stamp;
    lidar_to_local_origin.header.frame_id = pseudo_parent_frame_name;
    lidar_to_local_origin.child_frame_id = pseudo_child_frame_name;
    lidar_to_local_origin.transform.translation.x = lidar_pose_msg->pose.pose.position.x;
    lidar_to_local_origin.transform.translation.y = lidar_pose_msg->pose.pose.position.y;
    lidar_to_local_origin.transform.translation.z = lidar_pose_msg->pose.pose.position.z;
    lidar_to_local_origin.transform.rotation = lidar_pose_msg->pose.pose.orientation;

    broadcaster.sendTransform(lidar_to_local_origin);
}
                   
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pseudo_frame_tf_broadcaster");
    ros::NodeHandle node("~");

    node.param<std::string>("pseudo_child_frame_name", pseudo_child_frame_name, "pseudo_child_frame_name");
    node.param<std::string>("pseudo_parent_frame_name", pseudo_parent_frame_name, "pseudo_parent_frame_name");

    ros::Subscriber lidar_pose_sub = node.subscribe("lidar_pose", 10, &poseCallBack);

    ros::spin();

    return 0;
}