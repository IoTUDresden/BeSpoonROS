#!/usr/bin/env python 

import os
import subprocess
import rospy
from std_msgs.msg import String

default_file = "bespoon.jar"

"""
Start the bespoon tracker project in separate process and pipe output data. 

N.B.: 
We could publish it as Java ROS Node, we just wanted to avoid setup ros_java in our current configuration 
"""

def bespoon():
    rospy.init_node('bespoon_server', anonymous=True)    
    pub = rospy.Publisher('bespoon', String)
    # refresh rate     
    rate = rospy.Rate(10) # 0.1 = 10hz/100
    cmd = ['java', '-jar']

    # add java class path and jar file 
    jarfile_with_path = os.getenv('BESPOON_JAR_FILE')

    if jarfile_with_path is not None:       
        cmd.append(jarfile_with_path)
    else:
        cmd.append(default_file)
    
    # init a new proceess for bespoon
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    print("Process id: ", proc.pid)
    while proc.poll() is None: 
        output = proc.stdout.readline()
        #print(output.strip())
        data = output.strip('\n')
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
    retcode = proc.poll()
    print("Return code: ", retcode);
       

if __name__ == "__main__":    
    try:
        bespoon()
    except rospy.ROSInterruptException as e:
        print("Error: ", e)
