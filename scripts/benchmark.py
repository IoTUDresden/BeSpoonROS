#!/usr/bin/env python 

import rospy 
from std_msgs.msg import String 
from rosdata import RosData
from pyTrilater import cord_transform as ct 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Source: https://stackoverflow.com/a/21786287/1225337 
br = lambda x: '\x1b[0;30;41m {}\x1b[0m'.format(x)
bg = lambda x: '\x1b[6;30;42m {}\x1b[0m'.format(x)
rtext =  lambda x: '\033[91m {}\033[00m' .format(x)
gtext = lambda x: '\033[92m {}\033[00m' .format(x)

simple_ros = None
simple_bespoon = None 
threshold = 2.0 

def evaluate():      
    # Compare and print info     
    rxy = [simple_ros.position.x, simple_ros.position.y, simple_ros.position.z] if simple_ros is not None else [0,0,0]
    bxy = [simple_bespoon.position.x, simple_bespoon.position.y, simple_bespoon.position.z] if simple_bespoon is not None else [0,0,0]    
    # diff = ct.get_distance(rxy, bxy)    
    # s1 = gtext(diff1) if diff < threshold else rtext(diff)    
    print '--'*20
    print 'ROS simple coordinate: %s' % rxy 
    print 'Bespoon simple coordindate: %s' % bxy

def callback_ros(msg):
    global simple_ros
    simple_ros = msg
    # evaluate()

def callback_bespoon(msg):
    global simple_bespoon
    simple_bespoon = msg
    evaluate()

def init():  
    rospy.init_node('bespoon_eval', anonymous=True)
    rospy.Subscriber('simpleRosLocation', Pose, callback_ros)
    rospy.Subscriber('simpleBeSpoonLocation', Pose, callback_bespoon)    
    rospy.spin() 
    
def test():
    global simple_bespoon
    global simple_ros
    simple_bespoon = Pose( Point(1,1,0), Quaternion(0,0,0,1))
    simple_ros = Pose( Point(2,2,0), Quaternion(0,0,0,1))
    evaluate()

if __name__ == '__main__':
    init()
    # test() 

    
