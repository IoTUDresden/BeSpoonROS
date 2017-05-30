#!/usr/bin/env python 

import rospy
import traceback
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped, Pose


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Data=%s', data)
    ros_xyz = [data.position.x, data.position.y, data.position.z]
    # ROS position     
    data = {}
    data['ros'] = ros_xyz
    rospy.loginfo(data)


def subscribe():
    rospy.init_node('turtle_location_1', anonymous=True)
    rospy.Subscriber('tf_map_transform', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        print 'Starting turtle_location tracker'
        subscribe()
    except Exception as e:
        print("Error: ", e)
        print traceback.print_exc()