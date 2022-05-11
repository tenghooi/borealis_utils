#ifndef _PSEUDO_POINTCLOUD_TF_H_
#define _PSEUDO_POINTCLOUD_TF_H_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

class PointCloudTransformer
{
private:
    
    ros::Publisher transformed_pointcloud_pub_;
    ros::Subscriber map_pointcloud_sub_;

    tf::TransformListener map_to_pose_listener_;


public:

    PointCloudTransformer(ros::NodeHandle node);
    ~PointCloudTransformer();

    void MapCallBack(const sensor_msgs::PointCloud::ConstPtr& map_msg);


};

PointCloudTransformer::PointCloudTransformer(ros::NodeHandle node)
{
    transformed_pointcloud_pub_ = node.advertise<sensor_msgs::PointCloud>("transformed_map_pointcloud", 10);
    map_pointcloud_sub_ = node.subscribe<sensor_msgs::PointCloud>("map_pointcloud", 10, &PointCloudTransformer::MapCallBack, this);

}

PointCloudTransformer::~PointCloudTransformer()
{

}

void PointCloudTransformer::MapCallBack(const sensor_msgs::PointCloud::ConstPtr& map_msg)
{
    geometry_msgs::TransformStamped tf_stamped;
    sensor_msgs::PointCloud transformed_pointcloud;

    map_to_pose_listener_.transformPointCloud("pseudo_lidar_frame", *map_msg, transformed_pointcloud);

    transformed_pointcloud.header.frame_id = "uav1/t265_odom_frame";

    transformed_pointcloud_pub_.publish(transformed_pointcloud);
    
}

/*
void MapCallBack(const sensor_msgs::PointCloud::ConstPtr& map_msg)
{   
    geometry_msgs::TransformStamped tf_stamped;
    sensor_msgs::PointCloud transformed_cloud;

    map_tf_listener.transformPointCloud("pseudo_lidar_frame", *map_msg, transformed_cloud);

    //tf_stamped = tf_buffer.lookupTransform("aft_mapped", "camera_init", ros::Time(0));
    //transformed_map_pointcloud_pub.pub

}*/


#endif //_PSEUDO_POINTCLOUD_TF_H_