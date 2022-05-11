#include "pseudo_pointcloud_tf.h"

tf2_ros::Buffer tf_buffer;
//tf2_ros::TransformListener map_tf_listener(tf_buffer);

tf::TransformListener map_tf_listener;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pseudo_pointcloud_tf");
    ros::NodeHandle node("~");

    PointCloudTransformer transform_map_point_cloud(node);

    ros::spin();

    return 0;
}