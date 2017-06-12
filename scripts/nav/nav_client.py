#!/usr/bin/env python 


import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from bespoon.srv import * 

def nav_client():
    rospy.wait_for_service('nav_server')
    try: 
        service = rospy.ServiceProxy('nav_server', PoseType)

        pose = Pose( Point(0,0,0), Quaternion(0,0,0,1))
        pt = PoseTypeRequest()
        pt.pos = pose
        ret = service(pt)
        print ret 
    except rospy.ServiceException, e:
        print 'Error: %s' % e     
    

if __name__ == '__main__':
    nav_client() 