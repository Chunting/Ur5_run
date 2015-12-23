#!/usr/bin/env python
import rospy
import geometry_msgs.msg

target = None

def set_target(pose):
	global target
	target = pose
	print target

def listener():
	rospy.init_node('listener',anonymous = True)
	rospy.Subscriber("location", geometry_msgs.msg.Pose, set_target)
	rospy.spin()

if __name__ == '__main__':
	listener()
