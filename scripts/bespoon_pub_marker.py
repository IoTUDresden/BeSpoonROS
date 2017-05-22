#!/usr/bin/env python 

import rospy
import json 
import copy 
from std_msgs.msg import String 
from visualization_msgs.msg import MarkerArray

from rosdata import RosData
from tags_marker import BespoonMarker

class Bespoon(object):
    def __init__(self):
        rospy.init_node('bespoon_marker')
        self.rate = rospy.Rate(0.1)     # to do: update the rate 
        self.bespoon_data=dict() 
        self.marker_data=dict() 

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
                local_marker_data[str(t.tagId)] = [t.x, t.y, t.z]
            local_marker_data['anchor'] = ros_data.anchor
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
        # ToDo: process data here 
        
    def subscribe(self):
        # subscribe and publish again the subscribed data 
        rospy.Subscriber('bespoon', String, self.callback)

    def publish(self):
        self.topic = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        bpm = BespoonMarker()
        
        while not rospy.is_shutdown():            
            local_marker_data = copy.deepcopy(self.marker_data) 
            if isinstance(local_marker_data, dict) and local_marker_data.has_key('anchor'):                 
                anchor = local_marker_data.pop('anchor')                
                anchor if len(anchor) == 3 else anchor.append(0)
                self.topic.publish(bpm.prepare_anchor_marker_array(anchor))
                                            
            tags = bpm.prepare_tags_marker_array(local_marker_data)                        
            if  tags is not None:           # first data is zero point marker                 
                self.topic.publish(tags)
            self.rate.sleep()            

if __name__ == '__main__':
    try:
        b = Bespoon()
        # b.subscribe()
        # b.publish()        
        data = '{ "tags" : [{"tagId":3383,"x":2,"y":3,"z":0,"anchorDistance":0}, {"tagId":3384,"x":3,"y":2,"z":0,"anchorDistance":0}], "anchor": [1,1]}'
        # data = '{ "tags" : [], "anchor": [2,3]}'        
        b.marker_data= b.format_data_for_marker(data)         
        b.publish()        
    except Exception as e:
        print("Error:" , e)

