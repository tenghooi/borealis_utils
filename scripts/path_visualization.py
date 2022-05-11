#!/usr/bin/env python

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path 

from rospy.exceptions import ROSInterruptException

from collections import deque

def PoseStamped2Path(msg):

    msg.header.frame_id = "odom"
    opti_track_msg_queue.append(msg)

    pub = rospy.Publisher('opti_track_path', Path, queue_size=50)

    # ARITHMETIC FOR TRANSLATING initial pose to origin
    msg.pose.position.x -= 0.0345924145871
    msg.pose.position.y += 0.0138743830625
    msg.pose.position.z -= 0.272640327485

    path_msg = Path()
    path_msg.header = msg.header
    path_msg.poses = opti_track_msg_queue

    pub.publish(path_msg)

def LidarOdometry2Path(msg):

    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    #pose_msg.header.frame_id = "camera_init"
    # pose_msg.header.frame_id = "uav1/t265_odom_frame"
    pose_msg.header.frame_id = "odom"
    pose_msg.pose = msg.pose.pose
#TODO: ARITHMETIC FOR TRANSLATING POSE_MSG.POSE TO ORIGIN
    #pose_msg.pose.position.x -= 1.90293264389
    #pose_msg.pose.position.y += 1.10343837738
    #pose_msg.pose.position.z += 0.030658883974
    
    #msg.pose.pose.position.x -= 1.90293264389
    #msg.pose.pose.position.y += 1.10343837738
    #msg.pose.pose.position.z += 0.030658883974

    lidar_pose_msg_queue.append(pose_msg)

    path_msg = Path()
    path_msg.header = msg.header
    path_msg.poses = lidar_pose_msg_queue
    #path_msg.poses.append(pose_msg)

    pub = rospy.Publisher('lidar_uav_path', Path, queue_size=10)
    pub.publish(path_msg)

def CameraOdometry2Path(msg):

    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    #pose_msg.header.frame_id = "camera_init"
    # pose_msg.header.frame_id = "uav1/t265_odom_frame"
    pose_msg.header.frame_id = "odom"
    pose_msg.pose = msg.pose.pose

    t265_pose_msg_queue.append(pose_msg)

    path_msg = Path()
    path_msg.header = msg.header
    path_msg.poses = t265_pose_msg_queue

    pub = rospy.Publisher('t265_uav_path', Path, queue_size=20)
    pub.publish(path_msg)


def SubsAndVisualizePath():

    rospy.init_node('path_visualizer', anonymous=True)

    rospy.Subscriber("/vrpn_client_node/BorealisCoax/pose", PoseStamped, PoseStamped2Path, queue_size=100)
    rospy.Subscriber("/uav1/aft_mapped_to_init", Odometry, LidarOdometry2Path, queue_size=10)
    rospy.Subscriber("/uav1/t265/odom/sample", Odometry, CameraOdometry2Path, queue_size=50)

    rospy.spin()



# Create nav_msgs/Path with geometry_msgs/PoseStamped and nav_msgs/Odometry
if __name__ == '__main__':
    
    opti_track_msg_queue = deque(maxlen = 700)
    lidar_pose_msg_queue = deque(maxlen = 30)
    t265_pose_msg_queue = deque(maxlen = 500)

    try:
        SubsAndVisualizePath()

    except rospy.ROSInterruptException:
        pass

