#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

struct NodeParameter
{
  std::string pseudo_child_frame;
  std::string pseudo_parent_frame;
  std::string target_frame;
  std::string source_frame;

  double offset_x;
  double offset_y;
  double offset_z;
};

int main(int argc, char** argv){

  NodeParameter parameter;

  ros::init(argc, argv, "pseudo_frame_broadcaster");
  ros::NodeHandle node("~");

  node.param<std::string>("pseudo_child_frame_name", parameter.pseudo_child_frame, "pseudo_child_frame");
  node.param<std::string>("pseudo_parent_frame_name", parameter.pseudo_parent_frame, "pseudo_parent_frame");
  node.param<std::string>("target_frame_name", parameter.target_frame, "target_frame");
  node.param<std::string>("source_frame_name", parameter.source_frame, "source_frame");

  node.param<double>("offset_x", parameter.offset_x, 0.0);
  node.param<double>("offset_y", parameter.offset_y, 0.0);
  node.param<double>("offset_z", parameter.offset_z, 0.0);

  tf2_ros::TransformBroadcaster tf_broadcaster;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped broadcast_tf;

  broadcast_tf.header.frame_id = parameter.pseudo_parent_frame;
  broadcast_tf.child_frame_id = parameter.pseudo_child_frame;

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
      listen_tf = tf_buffer.lookupTransform(parameter.target_frame, parameter.source_frame, ros::Time(0));
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("%s",ex.what()); 
      ros::Duration(1.0).sleep();
      continue;
    }
    
    broadcast_tf.header.stamp = listen_tf.header.stamp;
    broadcast_tf.transform.translation.x = listen_tf.transform.translation.x + parameter.offset_x;
    broadcast_tf.transform.translation.y = listen_tf.transform.translation.y + parameter.offset_y;
    broadcast_tf.transform.translation.z = listen_tf.transform.translation.z + parameter.offset_z;
    broadcast_tf.transform.rotation = listen_tf.transform.rotation;

    tf_broadcaster.sendTransform(broadcast_tf);

    rate.sleep();
    
  }

};