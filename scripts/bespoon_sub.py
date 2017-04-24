#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Data= %s', data.data)

def subscribe():
    rospy.init_node('mysubscriber', anonymous=True)
    rospy.Subscriber('bespoon', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        print("Starting subscriber")
        subscribe()
    except Exception as e:
        print("Error: ", e)