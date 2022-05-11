
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
# One example application is:                             #
# It can be used to find initial offset of a pose of an   #
# object from its origin.                                 #
###########################################################

#!/usr/bin/env python

import rospy
from rospy.exceptions import ROSInterruptException

from std_msgs.msg import Header

from geometry_msgs.msg import Point, Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

import statistics
import math

def CalculateMeanPose(poses):

    sum_x = 0
    sum_y = 0
    sum_z = 0
    total_pose_count = len(poses)

    for pose in poses:
        sum_x += pose.point.x
        sum_y += pose.point.y
        sum_z += pose.point.z

    mean_pose = Point()
    mean_pose.x = sum_x / total_pose_count
    mean_pose.y = sum_y / total_pose_count
    mean_pose.z = sum_z / total_pose_count

    return mean_pose

def PoseAccumulator(msg, start_time):

    current_time = rospy.Time.now()

    time_pass = current_time - start_time

    mean_pose = Point()

    if time_pass < rospy.Time.from_sec(10.0):
        poses.append(msg.pose.point)
        mean_pose = CalculateMeanPose(poses)

    print(mean_pose) 

def SubscribePoseMsg():

    rospy.init_node("time_mean_pose", anonymous=True)

    start_time = rospy.Time.now()
    rospy.Subscriber("pose", PoseStamped, PoseAccumulator, start_time)

    rospy.spin()

if __name__ == '__main__':

    poses = []

    try:
        SubscribePoseMsg()

    except rospy.ROSInterruptException:
        pass