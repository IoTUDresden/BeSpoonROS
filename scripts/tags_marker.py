#!/usr/bin/env python 

import rospy
import copy 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


"""
Bespoon Tags and Anchor Marker
"""

class BespoonMarker(object):
    def __init__(self):
        # Create ros publisher 
        rospy.init_node('tags_marker')
        self.topic = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(0.1)
            
    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_rostime()  # ros.Time.now() 
        marker.lifetime = rospy.Duration()
        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "tags_marker"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Default settings 
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        # marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # return default marker 
        return marker

    def set_tag_label(self, marker, tid=0, pos=None, label='Tag'):    
        if not isinstance(pos,list) and len(pos) != 3: 
            raise Exception('Invalid parameters')
        
        # copy 
        tm = copy.deepcopy(marker)    

        # unpack list     
        p = self.get_point(pos)
        
        # set tags properties 
        tm.id = tid
        tm.text = label 
        tm.pose.position.x = p.x
        tm.pose.position.y = p.y;    
        tm.pose.position.z = 0.5

        tm.type = Marker.TEXT_VIEW_FACING    
        tm.scale.z = 0.2    
        tm.color.g = 1.0
        # tm.points.append(p)
        return tm 

        
    def get_point(self, coordinate=None):
        if coordinate is None: coordinate = [0,0,0]
        x,y,z = coordinate 
        p = Point()
        p.x = x
        p.y = y
        p.z = z 
        return p 


    def publish_tags(self, tags_coordinates):    
        if not isinstance(tags_coordinates, dict): raise Exception('Invalid parameters, expected: { id : [x,y,z]}')

        # Create marker, set tag values and add in Array 
        markers = MarkerArray()    
        point_marker = self.create_marker()
        # add point marker 
        markers.markers.append(point_marker)
        for tid, pos in  tags_coordinates.iteritems():
            # add text label marker 
            label_marker = self.set_tag_label(self.create_marker(), int(tid), pos, "Tag " + str(tid))
            markers.markers.append(label_marker)
            # add coordinate to point marker 
            point_marker.points.append(self.get_point(pos))        
        
        while not rospy.is_shutdown():
            # rospy.loginfo(msg)
            self.topic.publish(markers)
            self.rate.sleep()

    def publish_anchor(self, pos=None):
        if not isinstance(pos,list) and len(pos) != 3: 
            raise Exception('Invalid parameters, expected: [x,y,z]')

        markers = MarkerArray()    
        anchor_marker = self.create_marker()
        while not rospy.is_shutdown():             
            anchor_marker.points = list()
            anchor_marker.points.append(self.get_point(pos))
            markers.markers.append(anchor_marker)
            self.topic.publish(markers)
            self.rate.sleep()

if __name__ == '__main__':
    print 'Starting tags marker publisher'
    # tags coordinate in dict, { tagid = [position] } 
    tags_coord = {
        "3383" : [0, 0, 0], 
        "1107" : [5, 0, 0], 
        "2024" : [0, 3, 0], 
    }
    
    bm = BespoonMarker()
    bm.publish_tags(tags_coord)
    # bm.publish_anchor([2, 2, 0])