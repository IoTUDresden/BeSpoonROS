#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 

class Bespoon(object):
    def __init__(self):
        rospy.init_node('bespoon_marker')
        self.rate = rospy.Rate(0.1)
        self.bespoon_data='' 
                
    def callback(self, msg):
        """
        Subscriber data processing callback method 
        """
        self.bespoon_data = msg.data 
        # ToDo: process data here 
        
    def subscribe(self):
        # subscribe and publish again the subscribed data 
        rospy.Subscriber('bespoon', String, self.callback)

    def publish(self):
        self.topic = rospy.Publisher('bespoon2', String)  

        while not rospy.is_shutdown():
            msg = self.bespoon_data
            rospy.loginfo(msg)
            self.topic.publish(msg)
            self.rate.sleep()            

if __name__ == '__main__':
    try:
        b = Bespoon()
        b.subscribe()
        b.publish()        
    except Exception as e:
        print("Error:" , e)
        
