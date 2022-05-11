#!/usr/bin/env python

# Copyright (c) 2022, Teng Hooi Chan, Singapore University of Technology and Design
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


###########################################################
# This ROS package subscribes to poses of an object over  #
# a pre-defined amount of time and calculate a mean value #
# from the accumulated poses.                             #  
#                                                         #
# IMPORTANT: This module requires stamped message type    #
#                                                         #
# One example application is:                             #
# It can be used to find initial offset of a pose of an   #
# object from its origin.                                 #
###########################################################

import rospy
from rospy.exceptions import ROSInterruptException

from std_msgs.msg import Header

from geometry_msgs.msg import Point, Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

import math

def CalculateMeanPose(poses, mean_pose):

    sum_x = 0
    sum_y = 0
    sum_z = 0
    total_pose_count = len(poses)

    for pose in poses:
        sum_y += pose.pose.position.y
        sum_x += pose.pose.position.x
        sum_z += pose.pose.position.z

    mean_pose.x = sum_x / total_pose_count
    mean_pose.y = sum_y / total_pose_count
    mean_pose.z = sum_z / total_pose_count

    return mean_pose

def MeanPoseCallBack(msg, callback_args):

    time_frame_period = callback_args[0]
    mean_pose = callback_args[1]

    if len(poses) < 1:
        start_time = msg.header.stamp

    else:
        start_time = poses[0].header.stamp
    
    current_time = msg.header.stamp

    time_pass = current_time - start_time

    if time_pass < rospy.Duration.from_sec(time_frame_period):
        poses.append(msg)
        mean_pose = CalculateMeanPose(poses, mean_pose)

    print(mean_pose, len(poses)) 
    print(time_pass.to_sec())

def SubscribePoseMsg():

    rospy.init_node("time_mean_pose", anonymous=True)

    time_frame_period = rospy.get_param("~time_frame_secs", 5.0)
    msg_type = PoseStamped

    rospy.Subscriber("pose", msg_type, MeanPoseCallBack, (time_frame_period, mean_pose))

    rospy.spin()

if __name__ == '__main__':

    poses = []
    mean_pose = Point()

    try:
        SubscribePoseMsg()

    except rospy.ROSInterruptException:
        pass