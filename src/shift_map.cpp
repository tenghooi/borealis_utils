#include "shift_map.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shift_map");
    ros::NodeHandle node("~");

    PointCloudMapTransformer map_transformer(node);

    ros::spin();

    return 0;
}