#! /usr/bin/python3

"""

python - hello_world

"""
import rospy

if __name__ == "__main__":
	rospy.init_node("Hello")
	rospy.loginfo("Hello World!!!")
