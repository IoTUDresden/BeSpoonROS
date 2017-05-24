#!/usr/bin/env python 

import unittest

from bespoon_pub_marker import Bespoon 
from pyTrilater import cord_transform  as ct 

class TestBespoon(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls): 
        cls._bespoon = Bespoon()
    
    @classmethod
    def tearDownClass(cls):
            pass 

    def setUp(self): pass         
            
    def tearDown(self):  pass 

    def test_data_format(self):
        # data = '{ "tags" : [], "anchor": "{20,30}","anchorX":20,"anchorY":30}'        
        data = '{ "tags" : [{"tagId":3383,"x":2,"y":3,"z":0,"anchorDistance":0}, {"tagId":3384,"x":3,"y":2,"z":0,"anchorDistance":0}], "anchor": "{1; 1}","anchorX":1,"anchorY":1 }'
        expected = "{'3384': [0.03, 0.02, 0], '3383': [0.02, 0.03, 0], 'anchor': [0.01, 0.01, 0]}"
    
        marker_data = self._bespoon.format_data_for_marker(data)                  
        self.assertEqual(str(marker_data), expected)    

    def test_get_ros_xy(self):
        ct.ros_axis_scale_factor = 0.01
        e = [0.2, 0.3, 0]

        p = [20,30]
        a =  ct.get_ros_xy(p)
        self.assertEqual(a, e)

if __name__ == '__main__':    
    unittest.main()