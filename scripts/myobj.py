#!/usr/bin/evn python 

import json

class BObj:
    def toJson(self):
        return json.dumps(self, default=lambda o:o.__dict__, sort_keys=True, indent=4)

def test():
    obj = BObj()
    obj.tags = [1, 2, 3]
    obj.dist = [7, 8, 9]
    print(obj.toJson())


if __name__ == '__main__':
    test()