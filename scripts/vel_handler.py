#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Vel_handler():
    def __init__(self):
        self.vel_sub = rospy.Subscriber("/velocidade", Twist, self.velocidade_callback)
        self.dist_sub = rospy.Subscriber("/scan_min", Float64, self.laser_callback)
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.dist = 0.0
        self.velocidade = Twist()

    def laser_callback(self, data):
        self.dist = data.data

    def velocidade_callback(self, data):
        self.velocidade.linear.x = data.linear.x
        self.velocidade.angular.z = data.angular.z

        self.check_velocity()

    def check_velocity(self):
        if self.dist < 0.30:
            self.velocidade.linear.x = 0.0
            self.velocidade.angular.z = 0.0
            print("muito perto da parede")
        else:
            if abs(self.velocidade.linear.x) > 0.3:
                if self.velocidade.linear.x < 0.0:
                    self.velocidade.linear.x = 0.0
                else:
                    self.velocidade.linear.x = 0.3
            if abs(self.velocidade.angular.z) > 0.5:
                if self.velocidade.angular.z < 0.0:
                    self.velocidade.angular.z = -0.5
                else:
                    self.velocidade.angular.z = 0.5
        
        self.cmdvel_pub.publish(self.velocidade)
    
    def placeholder(self):
        pass

if __name__ == '__main__':
    rospy.init_node("vel_handler")
    rate = rospy.Rate(10)
    objt = Vel_handler()
    print("Initializing...")
    while not rospy.is_shutdown():
        objt.placeholder()
        rate.sleep()
