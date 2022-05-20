#ifndef _SHIFT_MAP_H_
#define _SHIFT_MAP_H_

#include <iostream>
#include <vector>
#include <string>

#include <sensor_msgs/PointCloud.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

struct NodeParameters
{
    std::string target_frame;

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
    node.param<std::string>("target_frame", parameters_.target_frame, "t265_odom_frame");
}

void PointCloudMapTransformer::MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg)
{

}

#endif //_SHIFT_MAP_H_