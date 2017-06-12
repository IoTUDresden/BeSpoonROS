#!/usr/bin/env python 

import sys  
import rospy
import actionlib 
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class Nav():

    def __init__(self):
        self.ros_node_init()        
    
    def ros_node_init(self, name='my_navigator'):
        if name is None: 
            raise Exception('name could not be empty')          
        rospy.init_node(name)          
        self.node_name=name
        self.nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.nav.wait_for_server()

    def send_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()        
        goal.target_pose.pose = Pose( Point(point[0],point[1],point[2]), Quaternion(0,0,0,1))

        self.nav.send_goal(goal) 
        # wait 30 sec for result 
        result = self.nav.wait_for_result(rospy.Duration(30))
        state = self.nav.get_state()

        ret = False 
        if result and state == GoalStatus.SUCCEEDED:
            ret = True
        else:
            self.nav.cancel_goal()
        return ret
    
    def stop(self):
        self.nav.cancel_goal()
        rospy.sleep(1)

if __name__ == '__main__':    
    try:       
        param = sys.argv[1:4]
        
        if ( len(param) < 3):
            print 'Error: Invalid parameters, required x,y,z'
            sys.exit()

        print 'Sending navigation goal'
        p = [ float(x) if '.' in x else int(x) for x in param ]
        nav = Nav()        
        ret = nav.send_goal(p)        

        msg = 'Goal Successful' if ret else 'Goal Failed'
        print msg 
        
        rospy.sleep(1)
    except Exception as e:
        print("Error:" , e)
