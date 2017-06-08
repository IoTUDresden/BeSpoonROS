#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Pose

def callback(data):
    rospy.loginfo('Position= %s', [data.position.x, data.position.y, data.position.z])

def subscribe():
    rospy.init_node('current_pos_node', anonymous=True)
    rospy.Subscriber('tf_map_transform', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        print("Starting subscriber")
        subscribe()
    except Exception as e:
        print("Error: ", e)