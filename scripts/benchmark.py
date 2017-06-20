#!/usr/bin/env python 

import rospy 
from std_msgs.msg import String 
from rosdata import RosData
from pyTrilater import cord_transform as ct 

# Source: https://stackoverflow.com/a/21786287/1225337 
br = lambda x: '\x1b[0;30;41m {}\x1b[0m'.format(x)
bg = lambda x: '\x1b[6;30;42m {}\x1b[0m'.format(x)
rtext =  lambda x: '\033[91m {}\033[00m' .format(x)
gtext = lambda x: '\033[92m {}\033[00m' .format(x)

threshold = 2.0 

def evaluate(msg):  
    data = RosData(msg.data)
    # Compare and print info 
    # data.turtle_ros_position, data.anchor_ros_position, data.turtle_custom_position, data.anchor_custom_position
    diff1 = ct.get_distance(data.turtle_ros_position, data.anchor_ros_position)
    diff2 = ct.get_distance(data.turtle_custom_position, data.anchor_custom_position)

    s1 = gtext(diff1) if diff1 < threshold else rtext(diff1)
    s2 = gtext(diff2) if diff2 < threshold else rtext(diff2)

    print '--'*20
    print 'ROS coordinate distance: ' + s1 , data.turtle_ros_position, data.anchor_ros_position
    print 'Custom coordindate distance: ' + s2, data.turtle_custom_position, data.anchor_custom_position

    pass 

def init():  
    rospy.init_node('bespoon_eval', anonymous=True)
    rospy.Subscriber('current_positions', String, evaluate)
    rospy.spin() 
    
def test():
    data = RosData()
    data.turtle_ros_position = [0.3, 2.99, 0]
    data.anchor_ros_position = [0.4, 3.1, 0]
    data.turtle_custom_position = [2, 4, 0]
    data.anchor_custom_position = [2, 50, 0]
    evaluate ( data.toJson())

if __name__ == '__main__':
    init()
    # test() 

    
