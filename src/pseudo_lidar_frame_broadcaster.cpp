#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pseudo_lidar_frame_broadcaster");
  ros::NodeHandle node("~");

  tf2_ros::TransformBroadcaster tf_broadcaster;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped broadcast_tf;

  
  broadcast_tf.header.frame_id = "uav1/pseudo_lidar_frame";
  broadcast_tf.child_frame_id = "uav1/t265_pose_frame";

/*
  broadcast_tf.header.stamp = ros::Time::now();
  broadcast_tf.transform.translation.x = 0;
  broadcast_tf.transform.translation.y = 0;
  broadcast_tf.transform.translation.z = 0;
  broadcast_tf.transform.rotation.w = 1;
  broadcast_tf.transform.rotation.x= 0;
  broadcast_tf.transform.rotation.y= 0;
  broadcast_tf.transform.rotation.z= 0; */


  ros::Rate rate(100.0);
  while (node.ok()){

    geometry_msgs::TransformStamped listen_tf;
  
    try
    {
      listen_tf = tf_buffer.lookupTransform("uav1/t265_pose_frame", "uav1/t265_odom_frame", ros::Time(0));
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("%s",ex.what()); 
      ros::Duration(1.0).sleep();
      continue;
    }
    
    broadcast_tf.header.stamp = listen_tf.header.stamp;
    broadcast_tf.transform = listen_tf.transform;

    tf_broadcaster.sendTransform(broadcast_tf);

    rate.sleep();
    
  }

};