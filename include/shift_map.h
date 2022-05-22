#ifndef _SHIFT_MAP_H_
#define _SHIFT_MAP_H_

#include <iostream>
#include <vector>
#include <string>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

struct NodeParameters
{
    std::string target_frame;

    geometry_msgs::Transform transform_vector;
};

class PointCloudMapTransformer
{
private:

    NodeParameters parameters_;

    ros::Publisher output_map_pub_;
    ros::Subscriber input_map_sub_;

    tf::TransformListener tf_listener_;

public:

    PointCloudMapTransformer(ros::NodeHandle& node);
    ~PointCloudMapTransformer();
    
    void SetNodeParameters(const ros::NodeHandle& node);
    void ToPCLPointCloud(const sensor_msgs::PointCloud& input_cloud, pcl::PointCloud<pcl::PointXYZ>& output_cloud);
    void ToROSPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, sensor_msgs::PointCloud& output_cloud);

    void MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg);

};

PointCloudMapTransformer::PointCloudMapTransformer(ros::NodeHandle& node)
{
    SetNodeParameters(node);

    input_map_sub_ = node.subscribe<sensor_msgs::PointCloud>("input_map_cloud", 10, &PointCloudMapTransformer::MapCallBack, this);
    output_map_pub_ = node.advertise<sensor_msgs::PointCloud>("output_map_cloud", 10);

}

PointCloudMapTransformer::~PointCloudMapTransformer() { }

void PointCloudMapTransformer::SetNodeParameters(const ros::NodeHandle& node)
{   
    node.param<std::string>("target_frame", parameters_.target_frame, "odom_frame");

    node.param<double>("transform_x", parameters_.transform_vector.translation.x, 0.0);
    node.param<double>("transform_y", parameters_.transform_vector.translation.y, 0.0);
    node.param<double>("transform_z", parameters_.transform_vector.translation.z, 0.0);
    parameters_.transform_vector.rotation.w = 1.0;
    parameters_.transform_vector.rotation.x = 0.0;
    parameters_.transform_vector.rotation.y = 0.0;
    parameters_.transform_vector.rotation.z = 0.0;
}

void PointCloudMapTransformer::ToPCLPointCloud(const sensor_msgs::PointCloud& input_cloud, pcl::PointCloud<pcl::PointXYZ>& output_cloud)
{
    pcl_conversions::toPCL(input_cloud.header, output_cloud.header);

    for (auto it = input_cloud.points.begin(); it != input_cloud.points.end(); ++it)
    {
        pcl::PointXYZ p;
        p.x = it->x;
        p.y = it->y;
        p.z = it->z;

        output_cloud.push_back(p); 
    }

    output_cloud.height = 1;
    output_cloud.width = output_cloud.points.size();
    output_cloud.is_dense = true;

}

void PointCloudMapTransformer::ToROSPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, sensor_msgs::PointCloud& output_cloud)
{
    pcl_conversions::fromPCL(input_cloud.header, output_cloud.header);

    for (auto it = input_cloud.points.begin(); it != input_cloud.points.end(); ++it)
    {
        geometry_msgs::Point32 p;
        p.x = it->x;
        p.y = it->y;
        p.z = it->z;

        output_cloud.points.push_back(p);
    }
}

void PointCloudMapTransformer::MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg)
{
    pcl::PointCloud<pcl::PointXYZ> raw_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud;
    
    ToPCLPointCloud(*input_msg, raw_pcl_cloud);
    pcl_ros::transformPointCloud(raw_pcl_cloud, transformed_pcl_cloud, parameters_.transform_vector);

    sensor_msgs::PointCloud output_ros_cloud;
    ToROSPointCloud(transformed_pcl_cloud, output_ros_cloud);

    // Set output frame_id to target_frame id
    output_ros_cloud.header.frame_id = parameters_.target_frame;

    output_map_pub_.publish(output_ros_cloud);
}

#endif //_SHIFT_MAP_H_