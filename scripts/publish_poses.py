#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
import numpy as np
import math
from visualization_msgs.msg import Marker
#initial_pose_x = rospy.get_param("~pose_x")
#initial_pose_y = rospy.get_param("~pose_y")
#initial_pose_z = rospy.get_param("~pose_z")


def move_fwd(velocity, t0, current_pose):
	t1 = rospy.Time.now().to_sec()
	new_pose = velocity*(t1 - t0) + current_pose
	current_pose = new_pose
	t0 = t1

	return current_pose, t0

def generateLaserScan(pointList, n, t):
    (X, Y, Z) = pointList
    l = len(Z)
    # Define the header block
    scanHeader = Header()
    scanHeader.seq = n
    scanHeader.stamp = t
    scanHeader.frame_id = '/os_sensor'
    # Generate point cloud
    points = []
    for i in range(l):
        p = Point32()
        p.x = X[i]
        p.y = Y[i]
        p.z = Z[i]
        points.append(p)
    # Prepare the pointcloud message
    pc = PointCloud()
    pc.header = scanHeader
    pc.points = points
    return pc

def publisher1():

	rospy.init_node('pose_publisher', anonymous=True)
	scanPublisher = rospy.Publisher('~pointcloud', PointCloud, queue_size=10)
	publi = rospy.Publisher('~pose', PoseWithCovarianceStamped, queue_size=1)

	rate = rospy.Rate(20) # Hz

	n = 1
	t0 = rospy.Time.now().to_sec()

	dest_fwd = rospy.get_param("~dest_fwd", 2)
	dest_bwd = rospy.get_param("~dest_bwd", -2)
	velocity = rospy.get_param("~velocity", 0)

	p = PoseWithCovarianceStamped()
	pcloud = PointCloud()
	p.header.frame_id = "/os_sensor"
	pcloud.header.frame_id = "/os_sensor"

	p.pose.pose.position.x = rospy.get_param("~pose_x", 0)
	p.pose.pose.position.y = rospy.get_param("~pose_y", 0)
	p.pose.pose.position.z = rospy.get_param("~pose_z", 0)

	pointList = []
	for i in range(3):
		pointList.append([])

	while not rospy.is_shutdown():
		
		current_pose = p.pose.pose.position.x
		
		p.pose.pose.position.x, t0 = move_fwd(velocity, t0, current_pose)

		if(p.pose.pose.position.x >= dest_fwd and velocity > 0):
			velocity = -abs(velocity)
		elif(p.pose.pose.position.x <= dest_bwd and velocity < 0):
			velocity = abs(velocity)
		
		# for i in np.arange(-1,1.1,0.5):
		# 	for j in np.arange(-1,1.1,0.5):
		# 		for k in np.arange(-1,1.1,0.5):
		# 			pointList[0].append(p.pose.pose.position.x + i)
		# 			pointList[1].append(p.pose.pose.position.y + j)
		# 			pointList[2].append(p.pose.pose.position.z + k)
		# laserScan = generateLaserScan(pointList, n, rospy.Time.now())
			
		# scanPublisher.publish(laserScan)
		# pointList[0] = []
		# pointList[1] = []
		# pointList[2] = []
		
		# print(velocity)

		n += 1
		publi.publish(p)

		rate.sleep()

if __name__ == '__main__':
    
	try:
		publisher1()

	except rospy:
		pass
