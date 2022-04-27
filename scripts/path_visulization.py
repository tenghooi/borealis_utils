#!/usr/bin/env python

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path 

from rospy.exceptions import ROSInterruptException

from collections import deque

def PoseStamped2Path(msg):

    msg.header.frame_id = "t265_odom_frame"
    target_msg_queue.append(msg)

    pub = rospy.Publisher('target_path', Path, queue_size=10)

    path_msg = Path()
    print(path_msg)
    path_msg.header = msg.header
    path_msg.poses = target_msg_queue

    pub.publish(path_msg)
    

def Odometry2Path(msg):

    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.header.frame_id = "camera_init"
    pose_msg.pose = msg.pose.pose
#TODO: ARITHMETIC FOR TRANSLATING POSE_MSG.POSE TO ORIGIN
    pose_msg.pose.position.x -= 1.90293264389
    pose_msg.pose.position.y += 1.10343837738
    pose_msg.pose.position.z += 0.030658883974
    
    #msg.pose.pose.position.x -= 1.90293264389
    #msg.pose.pose.position.y += 1.10343837738
    #msg.pose.pose.position.z += 0.030658883974

    pose_msg_queue.append(pose_msg)

    path_msg = Path()
    path_msg.header = msg.header
    path_msg.poses = pose_msg_queue
    #path_msg.poses.append(pose_msg)

    pub = rospy.Publisher('uav_path_history', Path, queue_size=10)
    pub.publish(path_msg)


def SubsAndConvertMsg():

    rospy.init_node('convert_msgs', anonymous=True)

    rospy.Subscriber("/final_trajectory_pose_0", PoseStamped, PoseStamped2Path, queue_size=10)
    rospy.Subscriber("/uav1/t265/odom/sample", Odometry, Odometry2Path, queue_size=10)

    rospy.spin()



# Create nav_msgs/Path with geometry_msgs/PoseStamped and nav_msgs/Odometry
if __name__ == '__main__':
    
    target_msg_queue = deque(maxlen = 500)
    # pose_msg_queue = deque(maxlen = 1000)
    pose_msg_queue = []

    try:
        SubsAndConvertMsg()

    except rospy.ROSInterruptException:
        pass

