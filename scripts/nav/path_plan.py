#!/usr/bin/env python 

import rospy 
import actionlib 
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap, GetPlan

def get_path_plan():
    rospy.init_node('test')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print 'Connected to the server'
    # /move_base/NavfnROS/make_plan 
    rospy.wait_for_service('/move_base/NavfnROS/make_plan')
    try:       

        start = PoseStamped()
        start.header.frame_id = 'map'
        # start.header.stamp = rospy.Time.now()        
        start.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1))


        end = PoseStamped()
        end.header.frame_id = 'map'             
        end.pose = Pose( Point(1,1,0), Quaternion(0,0,0,1))

        tolerance = 0.5

        getplan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        plan = getplan(start, end, tolerance)
        return plan 
    except Exception as e:
        print e

if __name__ == '__main__':
    print get_path_plan()
    
