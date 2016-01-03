#!/usr/bin/env python

# Node to animate and plot the trajectory of end effector

import roslib; roslib.load_manifest('two_link_planar_robot')
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque 

def animate_path(circle_length, rate_animate):
	
	animate_pub = rospy.Publisher('marker_visualization', Marker, queue_size=10)
	rate = rospy.Rate(rate_animate)
	
	marker = Marker()
	marker.header.frame_id = "base" # marker is referenced to base frame
	marker.header.stamp = rospy.Time.now()

	marker.type = marker.LINE_STRIP 
	marker.action = marker.ADD
	marker.scale.x = 0.01	# define the size of marker

	marker.color.a = 1.0
	marker.color.g = 1.0

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('base', 'end', rospy.Time())			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		p = Point() # Define a point array that takes in x, y and z values

		p.x = trans[0] 
		p.y = trans[1] 
		p.z = trans[2] 

		marker.points = deque(marker.points, circle_length) # Use deque to prevent list to increasing indefinitely; with this, after a certain circle_length, the list starts building from the beginning while replacing earlier elements
		marker.points.append(p) 
		
		animate_pub.publish(marker)		
		rate.sleep()

if __name__ == '__main__':
	rospy.init_node('animate_path')
	listener = tf.TransformListener()
	circle_length = rospy.get_param('~circle_length') # Get custom circle_length called during roslaunch
	rate_animate = rospy.get_param('~rate_animate') # Get custom rate caled during roslaunch
	animate_path(circle_length, rate_animate)