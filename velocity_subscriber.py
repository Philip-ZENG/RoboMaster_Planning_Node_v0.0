#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/velocity话题，消息类型geometry_msgs::Twist


import rospy
from geometry_msgs.msg import Twist


def velocityCallback(msg):
    rospy.loginfo("Robot velocity: x:%0.6f, y:%0.6f, a:%0.6f", msg.linear.x, msg.linear.y, msg.angular.z)

def velocity_subscriber():
	# ROS节点初始化
    rospy.init_node('velocity_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/robot1/velocity的topic，注册回调函数poseCallback
    rospy.Subscriber("/robot1/velocity", Twist, velocityCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    velocity_subscriber()
