#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
import tf

class Pose_handler():
    def __init__(self):
        self.sub_ponto = rospy.Subscriber("/clicked_point", PointStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        self.goal_pub = rospy.Publisher("/goal_atual", PointStamped, queue_size=10)

        self.has_started = False
        
        self.goals = []

        self.robot_pose = [0,0]
        self.robot_yaw = 0.0
        self.index = 0
    
    def goal_callback(self, data):
        self.goals.append(data.point.x)
        self.goals.append(data.point.y)

        if not self.has_started:
            self.goal_publisher()
            self.has_started = True 
    
    def odom_callback(self, data):
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1] = data.pose.pose.position.y
        
        orientation = (tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]))
        self.robot_yaw = orientation[2]

    def goal_publisher(self):
        if self.goals:
            if self.index >= len(self.goals):
                self.index = 0
    
            goal = PointStamped()
            goal.point.x = self.goals[self.index]
            goal.point.y = self.goals[self.index + 1]

            self.goal_pub.publish(goal)

    def goal_reached(self):
        if self.goals:
            if (abs(self.robot_pose[0] - self.goals[self.index]) < 0.3 and abs(self.robot_pose[1] - self.goals[self.index + 1]) < 0.3):
                self.index += 2
                self.goal_publisher()

                return True
            else:
                return False