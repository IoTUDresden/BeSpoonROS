#!/usr/bin/env python 

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
# Navigation in Groovy-ROS is built in rosbuild, not catkin 
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from pyTrilater import cord_transform as ct 

def publish():
    global ros_send_simple_goal_topic 
    rospy.init_node('nav_goal_node', anonymous=True)    
    topic = rospy.Publisher('simple_coord_nav_goal', Pose)    
    # ros_send_goal_topic = rospy.Publisher('move_base/goal', MoveBaseActionGoal) 
    ros_send_simple_goal_topic = rospy.Publisher('move_base_simple/goal', PoseStamped) 
    # while not rospy.is_shutdown():        
        # rospy.loginfo(msg)
        # topic.publish(msg)

def callback(data):
    global ros_send_simple_goal_topic 
    rospy.loginfo('Data:\n%s', data)
    # ToDo: convert and publish send_goal 
    goal = prepare_goal(data)    
    rospy.loginfo("Goal:\n%s", goal)
    # delegate the goal to final ROS topic 
    ros_send_simple_goal_topic.publish(goal)

def subscribe():
    # rospy.init_node('nav_goal', anonymous=True)
    rospy.Subscriber('simple_coord_nav_goal', Pose, callback)
    rospy.spin()

def prepare_goal(pose):    
    # transform custom xy_coordinate to ros_coordinate
    point = ct.get_ros_xy_from_simple_xy([pose.position.x, pose.position.y, pose.position.z])
    # create pose stamped for simple goal 
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()        
    goal.pose = Pose( Point(point[0],point[1],point[2]), pose.orientation)
    return goal 

# module global variable 
ros_send_simple_goal_topic = None 

if __name__ == '__main__':
    try:        
        publish()
        subscribe()
    except Exception as e:
        print("Error: ", e)