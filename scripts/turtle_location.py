#!/usr/bin/env python 

import rospy
import traceback
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped, Pose
from bespoon_pub_marker import Bespoon 

class Turtle(object):

    __bsp = Bespoon()

    def __init__(self):
        rospy.init_node('turtle_location_1', anonymous=True)
        self.bespoon_data=None
        self.ros_data=None
        self.location=dict()        
        pass 
    
    def callback(self, data):
        global bsp
        #rospy.loginfo(rospy.get_caller_id() + 'Data=%s', data)
        if isinstance(data, Pose): 
            # ROS position              
            self.ros_data = data      
            ros_xyz = [data.position.x, data.position.y, data.position.z]        
            self.location['ros'] = ros_xyz
        else: 
            # Bespoon data             
            self.bespoon_data = self.__bsp.format_data_for_marker(data.data)
            if self.bespoon_data.has_key('anchor'):  
                self.location['anchor'] = self.bespoon_data.get('anchor')
        # write to ros log             
        rospy.loginfo(self.location)

    def subscribe(self):        
        rospy.Subscriber('tf_map_transform', Pose, self.callback)
        rospy.spin()

    def subscribe2(self):        
        rospy.Subscriber('bespoon', String, self.callback)        

if __name__ == '__main__':
    try:
        print 'Starting turtle_location tracker'
        turtle = Turtle()
        turtle.subscribe2()
        turtle.subscribe()
    except Exception as e:
        print("Error: ", e)
        print traceback.print_exc()