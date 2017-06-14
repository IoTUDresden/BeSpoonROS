#!/usr/bin/evn python 

import json

class RosData(object):
    def __init__(self, json_data=None):
        if json_data is not None:
            self.__dict__ = json.loads(json_data)

    def toJson(self):
        return json.dumps(self, default=lambda o:o.__dict__, sort_keys=True) #  indent=4

    def fromJson(self, json_data):
        self.__dict__ = json.loads(json_data)
        return self 


if __name__ == '__main__':
    # test()
    data = '{ "tags" : [{"tagId":3383,"x":20,"y":30,"z":0,"anchorDistance":0}], "anchor": [2,3]}'
    r = RosData(data)    
    print r.anchor
    for t in r.tags: 
        mytag = t        
        print mytag['tagId']        
        temp = r.fromJson(json.dumps(mytag))
        print temp.tagId
    # print [r.tagId, r.x, r.y, r.z, r.anchorDistance]
    
