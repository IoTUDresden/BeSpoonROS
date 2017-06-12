#!/usr/bin/env python 


import rospy
from geometry_msgs.msg import Pose, Point
from bespoon.srv import *  

def callback(req):
    print req.pos          
    return PoseTypeResponse(req.pos) 
    
def nav_server():
    rospy.init_node('nav_server_node')
    service = rospy.Service('nav_server', PoseType, callback)
    print 'Custom coordinate navigation Server is started...'
    rospy.spin()

if __name__ == '__main__':
    nav_server()