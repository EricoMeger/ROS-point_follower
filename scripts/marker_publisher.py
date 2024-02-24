#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

#publishes a marker representing the robot goal
class publish_Marker():
    def __init__(self):
        self.sub_ponto = rospy.Subscriber("/goal_atual", PointStamped, self.point_callback)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        
        self.point_x = 0.0
        self.point_y = 0.0

        self.id = 0
        
    def point_callback(self, data):
        self.point_x = data.point.x
        self.point_y = data.point.y

        if self.id != 0:
            self.delete_point_marker()
        
        self.publish_point_marker()
    
    def delete_point_marker(self):
        marcador = Marker()
        #http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
        marcador.header.frame_id = "map"
        marcador.id = self.id - 1
        marcador.action = 3 #delete all
        marcador.pose.position.x = self.point_x
        marcador.pose.position.y = 0.0
        marcador.pose.position.z = 0.0
        marcador.pose.orientation.x = 0.0
        marcador.pose.orientation.y = 0.0
        marcador.pose.orientation.z = 0.0
        marcador.pose.orientation.w = 1.0
        marcador.scale.x = 0.0
        marcador.scale.y = 0.0
        marcador.scale.z = 0.0
        marcador.color.a = 0.0 # Don't forget to set the alpha!
        marcador.color.r = 0.8
        marcador.color.g = 0.5
        marcador.color.b = 0.0

        self.marker_pub.publish(marcador)

    def publish_point_marker(self):
        marcador = Marker()
        #http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
        marcador.header.frame_id = "map"
        marcador.header.stamp = rospy.Time.now()
        marcador.id = self.id
        marcador.type = 2#type sphere
        marcador.action = 0 #add 
        marcador.pose.position.x = self.point_x
        marcador.pose.position.y = self.point_y
        marcador.pose.position.z = 0.0
        marcador.pose.orientation.x = 0.0
        marcador.pose.orientation.y = 0.0
        marcador.pose.orientation.z = 0.0
        marcador.pose.orientation.w = 1.0
        marcador.scale.x = 0.3
        marcador.scale.y = 0.3
        marcador.scale.z = 0.3
        marcador.color.a = 1.0 # Don't forget to set the alpha!
        marcador.color.r = 0.8
        marcador.color.g = 0.5
        marcador.color.b = 0.0

        self.marker_pub.publish(marcador)
        self.id += 1
    
    def placeholder(self):
        pass

if __name__ == '__main__':
    rospy.init_node("marker_publisher")
    rate = rospy.Rate(10)
    objt = publish_Marker()
    print("Initializing...")
    while not rospy.is_shutdown():
        objt.placeholder()
        rate.sleep()
        

