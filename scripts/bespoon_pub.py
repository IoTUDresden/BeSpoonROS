#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 
import bespoon_java_server

def publish():
    topic = rospy.Publisher('bespoon', String)
    rospy.init_node('mypublisher')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = "Testing data"
        rospy.loginfo(msg)
        topic.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # publish()
        bespoon_java_server.bespoon()
    except Exception as e:
        print("Error:" , e)
        
