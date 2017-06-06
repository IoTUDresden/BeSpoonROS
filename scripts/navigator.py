#!/usr/bin/env python 

import rospy
import actionlib 
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class Nav():

    def __init__(self):
        self.ros_node_init()
        self.nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.nav.wait_for_server()
    
    def ros_node_init(self, name='my_navigator'):
        if name is None: 
            raise Exception('topic and name could not be empty')
        #  topic = visualization_marker_array, MarkerArray      
        rospy.init_node(name)          
        self.node_name=name

    def send_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()        
        goal.target_pose.pose = Pose( Point(point[0],point[1],0), Quaternion(0,0,0,1))

        self.nav.send_goal(goal) 
        result = self.nav.wait_for_result()
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
        nav = Nav()
        ret = nav.send_goal([1,1])
        if ret: 
            rospy.loginfo('Goal Successful')
        else: 
            rospy.loginfo('Goal Failed')
        rospy.sleep(1)
    except Exception as e:
        print("Error:" , e)
