#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布robot1/pose话题，消息类型geometry_msgs::PoseStamped

import rospy
from geometry_msgs.msg import PoseStamped


def pose_publisher():
	# ROS节点初始化
    rospy.init_node('pose_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/robot1/own_pose的topic，消息类型为geometry_msgs::PoseStamped，队列长度10
    robot_own_pose_pub = rospy.Publisher('/robot1/own_pose', PoseStamped, queue_size=10)
    # 创建一个Publisher，发布名为/robot1/ally_pose的topic，消息类型为geometry_msgs::PoseStamped，队列长度10
    robot_ally_pose_pub = rospy.Publisher('/robot1/ally_pose', PoseStamped, queue_size=10)
    # 创建一个Publisher，发布名为/robot1/enemy1_pose的topic，消息类型为geometry_msgs::PoseStamped，队列长度10
    robot_enemy1_pose_pub = rospy.Publisher('/robot1/enemy1_pose', PoseStamped, queue_size=10)
    # 创建一个Publisher，发布名为/robot1/enemy2_pose的topic，消息类型为geometry_msgs::PoseStamped，队列长度10
    robot_enemy2_pose_pub = rospy.Publisher('/robot1/enemy2_pose', PoseStamped, queue_size=10)


	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化geometry_msgs::PoseStamped类型的消息
        own_pos_msg = PoseStamped()
        own_pos_msg.pose.position.x = 0
        own_pos_msg.pose.position.y = 200
        own_pos_msg.pose.orientation.x = 0
        own_pos_msg.pose.orientation.y = 0

        ally_pos_msg = PoseStamped()
        ally_pos_msg.pose.position.x = 200
        ally_pos_msg.pose.position.y = 300
        ally_pos_msg.pose.orientation.x = 0
        ally_pos_msg.pose.orientation.y = 0

        
        enemy1_pos_msg = PoseStamped()
        enemy1_pos_msg.pose.position.x = 500
        enemy1_pos_msg.pose.position.y = 400
        enemy1_pos_msg.pose.orientation.x = 0
        enemy1_pos_msg.pose.orientation.y = 0

        
        enemy2_pos_msg = PoseStamped()
        enemy2_pos_msg.pose.position.x = 700
        enemy2_pos_msg.pose.position.y = 150
        enemy2_pos_msg.pose.orientation.x = 0
        enemy2_pos_msg.pose.orientation.y = 0



		# 发布消息
        robot_own_pose_pub.publish(own_pos_msg)
        rospy.loginfo("Publsh robot positoin info [x:%0.6f , y: %0.6f ]", own_pos_msg.pose.position.x, own_pos_msg.pose.position.y)
        # rospy.loginfo("Publsh robot orientation info [x:%0.6f , y: %0.6f ]", pos_msg.pose.orientation.x, pos_msg.pose.orientation.y)

        # 发布消息
        robot_ally_pose_pub.publish(ally_pos_msg)
        rospy.loginfo("Publsh robot positoin info [x:%0.6f , y: %0.6f ]", ally_pos_msg.pose.position.x, ally_pos_msg.pose.position.y)
        
        # 发布消息
        robot_enemy1_pose_pub.publish(enemy1_pos_msg)
        rospy.loginfo("Publsh robot positoin info [x:%0.6f , y: %0.6f ]", enemy1_pos_msg.pose.position.x, enemy1_pos_msg.pose.position.y)

        # 发布消息
        robot_enemy2_pose_pub.publish(enemy2_pos_msg)
        rospy.loginfo("Publsh robot positoin info [x:%0.6f , y: %0.6f ]", enemy2_pos_msg.pose.position.x, enemy2_pos_msg.pose.position.y)

		# 按照循环频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass

