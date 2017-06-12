#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Pose, Point, Quaternion

def publish():
    rospy.init_node('nav_goal_node', anonymous=True)    
    topic = rospy.Publisher('custom_coord_nav_goal', Pose)    
    # while not rospy.is_shutdown():        
        # rospy.loginfo(msg)
        # topic.publish(msg)

def callback(data):
    rospy.loginfo('Data=%s', data)

def subscribe():
    # rospy.init_node('nav_goal', anonymous=True)
    rospy.Subscriber('custom_coord_nav_goal', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # example: rostopic pub -1 custom_coord_nav_goal geometry_msgs/Pose '{position : { x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}'                              
        publish()
        subscribe()
    except Exception as e:
        print("Error: ", e)