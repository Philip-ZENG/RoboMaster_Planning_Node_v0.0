#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布robot1/target话题，消息类型geometry_msgs::PoseStamped

import rospy
from geometry_msgs.msg import PoseStamped


def target_publisher():
	# ROS节点初始化
    rospy.init_node('target_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/robot1/target的topic，消息类型为geometry_msgs::PoseStamped，队列长度10
    robot_pose_pub = rospy.Publisher('/robot1/target', PoseStamped, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化geometry_msgs::PoseStamped类型的消息
        pos_msg = PoseStamped()
        pos_msg.pose.position.x = 300
        pos_msg.pose.position.y = 100
        pos_msg.pose.orientation.x = 0
        pos_msg.pose.orientation.y = 0

		# 发布消息
        robot_pose_pub.publish(pos_msg)
        rospy.loginfo("Publsh robot target positoin info [x:%0.6f , y: %0.6f ]", pos_msg.pose.position.x, pos_msg.pose.position.y)
        rospy.loginfo("Publsh robot target orientation info [x:%0.6f , y: %0.6f ]", pos_msg.pose.orientation.x, pos_msg.pose.orientation.y)

		# 按照循环频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass

