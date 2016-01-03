#!/usr/bin/env python

# Node to publish joint_states 

import roslib; roslib.load_manifest('two_link_planar_robot')
import rospy
from sensor_msgs.msg import JointState
from math import cos, sin, pi, sqrt, atan2

# publish joint_states
def update_state(rate_joint):
	joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(rate_joint)
	joint_state = JointState()
	
	while not rospy.is_shutdown():
		joint_state.header.stamp = rospy.Time.now()
		t = rospy.get_time()
		joint_state.name = ['joint1','joint2']
		joint_state.position = [get_state(t)[0], get_state(t)[1]] # update joint_states with time
		joint_pub.publish(joint_state)
		
		rate.sleep()

# Using inverse kinematics, obtain joint angles
def get_state(t):
	x = (0.5 * cos(2*pi*t/5.0)) + 1.25
	y = 0.5 * sin(2*pi*t/5.0)
	l1 = 1.0
	l2 = 1.0

	# Calculate theta_1 & theta_2

	theta_2 = atan2((+sqrt(1-((x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2))**2)), ((x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)))

	k1 = l1 + l2*cos(theta_2)
	k2 = l2*sin(theta_2)

	theta_1 = atan2(y,x) - atan2(k2,k1)

	return [theta_1, theta_2]

if __name__ == '__main__':
	rospy.init_node('update_state')
	rate_joint = rospy.get_param('~rate_joint') # Rate parameter called here
	update_state(rate_joint)
	

