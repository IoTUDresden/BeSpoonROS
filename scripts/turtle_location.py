#!/usr/bin/env python 

import rospy
import traceback
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from bespoon_pub_marker import Bespoon 
from pyTrilater import cord_transform as ct 

class Turtle(object):

    __bsp = Bespoon()

    def __init__(self):        
        self.ros_data=None
        self.bespoon_data=None        
        self.rxy = list()       # turtle current location from map 
        self.bxy = list()       # bespoon anchor location from bespoon
                    
    def callback(self, data):                
        if isinstance(data, Pose): 
            # turtle ros location 
            self.ros_data = data      
            self.rxy = [data.position.x, data.position.y, data.position.z]                    
        else: 
            # anchor location  
            self.bespoon_data = self.__bsp.format_data_for_marker(data.data)
            if self.bespoon_data.has_key('anchor'):  
                self.bxy = self.bespoon_data.get('anchor')                     
        # publish all data again 
        self.publish()

    def subscribe_ros_position(self):        
        rospy.Subscriber('tf_map_transform', Pose, self.callback)
        # rospy.spin()

    def subscribe_anchor_position(self):        
        rospy.Subscriber('bespoon', String, self.callback)
        # rospy.spin()       

    def simple_coord_publisher(self):
        rospy.init_node('simple_position_node', anonymous=True)        
        self.topic_simple_ros = rospy.Publisher('simpleRosLocation', Pose)  
        self.topic_simple_anchor = rospy.Publisher('simpleBeSpoonLocation', Pose)  

    def publish(self):
        # Use simple_xy conversion 
        simple_rxy = ct.get_simple_xy_from_ros_xy(self.rxy) if len(self.rxy) > 1 else [None, None, None]
        simple_bxy = ct.get_simple_xy_from_ros_xy(self.bxy) if len(self.bxy) > 1 else [None, None, None]

        rpose = Pose( Point (simple_rxy[0], simple_rxy[1], simple_rxy[2]) , self.ros_data.orientation)
        bpose = Pose( Point (simple_bxy[0], simple_bxy[1], simple_bxy[2]) , Quaternion(0,0,0,1))

        # publish pose information in simple coordinate 
        self.topic_simple_ros.publish(rpose)
        self.topic_simple_anchor.publish(bpose)


if __name__ == '__main__':
    try:
        print 'Starting turtlebot simple location publisher'
        turtle = Turtle()
        turtle.simple_coord_publisher()
        turtle.subscribe_ros_position()
        turtle.subscribe_anchor_position()
        rospy.spin()
    except Exception as e:
        print("Error: ", e)
        print traceback.print_exc()