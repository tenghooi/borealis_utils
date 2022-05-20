#ifndef _SHIFT_MAP_H_
#define _SHIFT_MAP_H_

#include <iostream>
#include <vector>
#include <string>

#include <sensor_msgs/PointCloud.h>
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
    void toPCL(const sensor_msgs::PointCloud& ros_cloud, pcl::PointCloud<P);
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

void PointCloudMapTransformer::FromROSPointCloudMsg(const sensor_msgs::PointCloud& input_cloud)
{

}

void PointCloudMapTransformer::MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg)
{
    
}

#endif //_SHIFT_MAP_H_