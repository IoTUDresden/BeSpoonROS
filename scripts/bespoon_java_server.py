#!/usr/bin/env python 

"""
BeSpoon(java) anchor tracker run as ROS python topic and publish 

set CLASSPATH=<bespoon-java-jar-path>

"""

import rospy
from std_msgs.msg import String
import subprocess
import os

default_file = "bespoon.jar"

def bespoon():
    pub = rospy.Publisher('bespoon', String)
    # queue_size not supported in groovy 
    #pub = rospy.Publisher('bespoon', String, queue_size=10)    
    rospy.init_node('bespoon_server', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cmd = ['java', '-jar']

    # add java class path and jar file 
    jarfile_with_path = os.getenv('FILE_NAME')

    if jarfile_with_path != None:       
        cmd.append(jarfile_with_path)
    else:
        cmd.append(default_file)
    ###########

    # init a new proceess for bespoon
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    print("Process id: ", proc.pid)
    while proc.poll() is None: 
        output = proc.stdout.readline()
        #print(output.strip())
        data = "{ %s }" % output.strip('\n')
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
    retcode = proc.poll()
    print("Return code: ", retcode);

if __name__ == "__main__":    
    try:
        bespoon()
    except rospy.ROSInterruptException:
        pass
