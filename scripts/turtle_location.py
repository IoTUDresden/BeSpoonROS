#!/usr/bin/env python 

import rospy
import traceback
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped, Pose
from bespoon_pub_marker import Bespoon 
from rosdata import RosData 
from pyTrilater import cord_transform as ct 

class Turtle(object):

    __bsp = Bespoon()

    def __init__(self):        
        self.ros_data=None
        self.bespoon_data=None        
        self.rxy = list()       # turtle current position from map 
        self.bxy = list()       # anchor current position from bespoon
                    
    def callback(self, data):        
        #rospy.loginfo(rospy.get_caller_id() + 'Data=%s', data)

        if isinstance(data, Pose): 
            # turtle ros position data
            self.ros_data = data      
            self.rxy = [data.position.x, data.position.y, data.position.z]                    
        else: 
            # anchor position data 
            self.bespoon_data = self.__bsp.format_data_for_marker(data.data)
            if self.bespoon_data.has_key('anchor'):  
                self.bxy = self.bespoon_data.get('anchor')             
        # rospy.loginfo()
        # publish all data again 
        self.publish()

    def subscribe_ros_position(self):        
        rospy.Subscriber('tf_map_transform', Pose, self.callback)
        # rospy.spin()

    def subscribe_anchor_position(self):        
        rospy.Subscriber('bespoon', String, self.callback)
        # rospy.spin()       

    def position_publisher(self):
        rospy.init_node('all_position_node', anonymous=True)
        self.topic = rospy.Publisher('current_positions', String)  

    def publish(self):
        data = RosData()
        data.turtle_ros_position = self.rxy 
        data.anchor_ros_position = self.bxy 
        data.turtle_custom_position = ct.get_simple_xy_from_ros_xy(self.rxy) if len(self.rxy) > 1 else list()
        data.anchor_custom_position = ct.get_simple_xy_from_ros_xy(self.bxy) if len(self.bxy) > 1 else list()
        # rospy.loginfo(data.toJson())        
        self.topic.publish(data.toJson())

if __name__ == '__main__':
    try:
        print 'Starting turtlebot position publisher'
        turtle = Turtle()
        turtle.position_publisher()
        turtle.subscribe_ros_position()
        turtle.subscribe_anchor_position()
        rospy.spin()
    except Exception as e:
        print("Error: ", e)
        print traceback.print_exc()