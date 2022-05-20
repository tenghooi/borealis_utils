#ifndef _SHIFT_MAP_H_
#define _SHIFT_MAP_H_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
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

    PointCloudMapTransformer(ros::NodeHandle node);
    ~PointCloudMapTransformer();
    
    void MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg);

};

PointCloudMapTransformer::PointCloudMapTransformer(ros::NodeHandle node)
{

}

PointCloudMapTransformer::~PointCloudMapTransformer() { }

void MapCallBack(const sensor_msgs::PointCloud::ConstPtr& input_msg)
{
    
}

#endif //_SHIFT_MAP_H_