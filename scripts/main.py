#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pose_handler import Pose_handler
import math

class Navigate():
    def __init__(self):
        self.manager = Pose_handler()
        
        self.goal_sub = rospy.Subscriber("/goal_atual", PointStamped, self.goal_handler)
        self.vel_pub = rospy.Publisher("/velocidade", Twist, queue_size=10)

        self.goal = []

        self.kp_linear = 0.23
        self.kp_angular = 0.39
        self.kd_linear = 0.4
        self.kd_angular = 0.48

        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
        
        self.vel = Twist()

        self.reached = False
        self.rotate_in_axis = False
        self.start = True

    def goal_handler(self, data):
        if self.start:
            self.goal.append(data.point.x)
            self.goal.append(data.point.y)
            self.start = False
        else:
            self.goal[0] = data.point.x 
            self.goal[1] = data.point.y
        
        print("goal atual:", self.goal)

    def check_angle(self, goal_angle):
        if abs(goal_angle) > math.pi/2 and abs(goal_angle) < 3 * math.pi / 2:
            self.rotate_in_axis = True
            
        if abs(goal_angle) < 0.2:
            self.rotate_in_axis = False

    def calc_linear_vel(self):
        linear_diff = math.sqrt((self.goal[0]-self.manager.robot_pose[0])**2 + (self.goal[1]-self.manager.robot_pose[1])**2)
        linear_diff_error = linear_diff - self.last_linear_error

        self.vel.linear.x = linear_diff * self.kp_linear + linear_diff_error * self.kd_linear
        self.last_linear_error = linear_diff

    def calc_angular_vel(self):
        delta_x = self.goal[0] - self.manager.robot_pose[0]
        delta_y = self.goal[1] - self.manager.robot_pose[1]
        goal_angle = math.atan2(delta_y, delta_x)
        
        angular_diff = goal_angle - self.manager.robot_yaw
        angular_diff_error = angular_diff - self.last_angular_error

        self.check_angle(angular_diff)

        self.vel.angular.z = angular_diff * self.kp_angular + angular_diff_error * self.kd_angular
        self.last_angular_error = angular_diff

    def main(self):
        if not self.manager.goal_reached() and not self.start:
            self.calc_angular_vel()

            if not self.rotate_in_axis:
                #print("ok")
                self.calc_linear_vel()
            else:
                self.vel.linear.x = 0.0
        

        self.vel_pub.publish(self.vel)

if __name__ == '__main__':
    rospy.init_node("Trajetoria")
    rate = rospy.Rate(10)
    objt = Navigate()
    print("Initializing...")
    while not rospy.is_shutdown():
        objt.main()
        rate.sleep()


