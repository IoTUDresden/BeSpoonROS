#!/usr/bin/env python 

import rospy
import json 
import copy 
import traceback
from std_msgs.msg import String 
from visualization_msgs.msg import MarkerArray

from rosdata import RosData
from tags_marker import BespoonMarker
from pyTrilater import cord_transform as ct 

class Bespoon(object):
    def __init__(self):       
        self.bespoon_data=dict() 
        self.marker_data=dict() 

    def ros_node_init(self, name='bespoon_marker'):
        if name is None: 
            raise Exception('topic and name could not be empty')
        #  topic = visualization_marker_array, MarkerArray      
        rospy.init_node(name)          
        self.node_name=name
        self.rate = rospy.Rate(10)           # 1Hz = 1 per second         

    """
    Format the bespoon topic data for marker visualization 
    """    
    def format_data_for_marker(self, data):        
        local_marker_data = dict()        
        try: 
            # convert json string to ros data 
            ros_data = RosData(data)                
            for tag in ros_data.tags: 
                # convert tag dict to rosdata             
                t = RosData(json.dumps(tag))
                # transform bespoon to ros coordinate 
                local_marker_data[str(t.tagId)] =  ct.get_ros_xy([t.x, t.y, t.z])
            # anchor position list 
            local_marker_data['anchor'] = ct.get_ros_xy([ ros_data.anchorX, ros_data.anchorY ]) 
        except Exception as e:
            print "Error: json data parsing error\n", e            
        # print marker_data
        return local_marker_data
                
    def callback(self, msg):
        """
        Subscriber data processing callback method 
        """
        self.bespoon_data = msg.data                 
        self.marker_data= self.format_data_for_marker(msg.data)                 
        # rospy.loginfo(self.marker_data)
        
    def subscribe(self):
        # subscribe and publish again the processed data 
        rospy.Subscriber('bespoon', String, self.callback)

    def publish(self):
        self.topic = rospy.Publisher('visualization_marker_array', MarkerArray)
        bpm = BespoonMarker()
        print "Bespoon marker publishing..."
        
        while not rospy.is_shutdown():                        
            local_marker_data = copy.deepcopy(self.marker_data)             
            if isinstance(local_marker_data, dict): 
                if local_marker_data.has_key('anchor'):  
                    anchor = local_marker_data.pop('anchor')                
                    anchor if len(anchor) == 3 else anchor.append(0)
                    self.topic.publish(bpm.prepare_anchor_marker_array(anchor))            
                
                tags = bpm.prepare_tags_marker_array(local_marker_data)                        
                if  tags is not None:           
                    self.topic.publish(tags)
            self.rate.sleep()            

if __name__ == '__main__':    
    try:
        # transform constants 
        ct.ros_axis_scale_factor = 0.01
        ct.bespoon_plane_angle_with_ros_plane = 0.0
        ct.bespoon_center_in_ros_plane = [0, 0]

        b = Bespoon()   
        b.ros_node_init();                          
        b.subscribe()
        b.publish()               
    except Exception as e:
        print("Error:" , e)
        print traceback.print_exc()

