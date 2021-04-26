## 整合与planning node相关的3个 publisher-subscriber结构
## velocity_publisher
## target_subscriber
## pose_subscriber

"""
    ## APF的更新频率要高于Astar的更新频率
    # APF在每一次Astar给出路线后，APF更新5次（暂定）
    # Astar 更新频率是2 （每0.5秒更新一次）
    # APF 的更新频率是10 （每0.1秒更新一次）
    # APF更新频率与planning节点给base节点发送速度指令的频率
    # Astar更新频率与决策节点给planning节点发送目的地坐标的频率、感知节点给planning节点车辆位置信息的频率、地图更新频率相同相同
"""


import rospy
import a_star
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


car_position = a_star.CarPosition()
target = a_star.Target()
pathList = []
planner = a_star.Planner()


def planning_node():
    # ROS节点初始化
    rospy.init_node('planning_node', anonymous=True)

	# 创建一个Publisher，发布名为/robot1/velocity的topic，消息类型为geometry_msgs::Twist，队列长度10
    robot_vel_pub = rospy.Publisher('/robot1/velocity', Twist, queue_size=10)

	# 设置循环的频率
    rate = rospy.Rate(2) 


    while not rospy.is_shutdown():
        
        # 接收信息包括：四个车辆的位置、移动目标
        own_pos = rospy.wait_for_message("/robot1/own_pose", PoseStamped)
        car_position.own = (own_pos.pose.position.x, own_pos.pose.position.y,60,45)
        rospy.loginfo("Robot position: x:%0.6f, y:%0.6f", own_pos.pose.position.x, own_pos.pose.position.y)
        ally_pos = rospy.wait_for_message("/robot1/ally_pose", PoseStamped)
        car_position.ally = (ally_pos.pose.position.x, ally_pos.pose.position.y,60,45)
        rospy.loginfo("Ally position: x:%0.6f, y:%0.6f", ally_pos.pose.position.x, ally_pos.pose.position.y)
        enemy1_pos = rospy.wait_for_message("/robot1/enemy1_pose", PoseStamped)
        car_position.enemy1 = (enemy1_pos.pose.position.x, enemy1_pos.pose.position.y,60,45)
        rospy.loginfo("Enemy1 position: x:%0.6f, y:%0.6f", enemy1_pos.pose.position.x, enemy1_pos.pose.position.y)
        enemy2_pos = rospy.wait_for_message("/robot1/enemy2_pose", PoseStamped)
        car_position.enemy2 = (enemy2_pos.pose.position.x, enemy2_pos.pose.position.y,60,45)
        rospy.loginfo("Enemy2 position: x:%0.6f, y:%0.6f", enemy2_pos.pose.position.x, enemy2_pos.pose.position.y)
        target_msg = rospy.wait_for_message("/robot1/target", PoseStamped)
        rospy.loginfo("Robot target position: x:%0.6f, y:%0.6f", target_msg.pose.position.x, target_msg.pose.position.y)
        target.x = target_msg.pose.position.x
        target.y = target_msg.pose.position.y
        

        planner.map_initialization() # 初始化静态障碍物
        planner.update_map(car_position) # 依据四个车辆的位置，更新地图
        pathList = planner.astar_plan(car_position.own[0],car_position.own[1],target.x,target.y) # astar规划出全局路径

	
        for j in range(0,5):
            # 执行局部路径规划
            velocity = a_star.Velocity()
            velocity = planner.APF_plan(pathList,j) # APF给出局部移动的速度信息，将全局路径分为五段，依此次输出五个速度指令

            # 更新geometry_msgs::Twist类型的消息
            vel_msg = Twist()
            vel_msg.linear.x = velocity.x
            vel_msg.linear.y = velocity.y
            vel_msg.angular.z = velocity.a

            # 发布消息
            robot_vel_pub.publish(vel_msg)
            rospy.loginfo("Publish robot velocity command[x:%0.6f m/s, y: %0.6f m/s, %0.6f rad/s]", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z)
            time.sleep(0.05)

		# 按照循环频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        # ROS节点初始化
        rospy.init_node('planning_node', anonymous=True)
    
        # 构建1个publisher以及2个subscriber
        planning_node()
    
    except rospy.ROSInterruptException:
        pass
