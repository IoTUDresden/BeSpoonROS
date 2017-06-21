#!/usr/bin/env python 

from pyTrilater import cord_transform as ct 

def read_xy():
    x = input("Enter X:")
    y = input("Enter Y:")
    return (x,y)


def main(): 
    print 'ROS-MyCord Conversion'
    sf = input('Scale: ')
    while True:
        c = input("Select (1. ROS-to-MyCord, 2. MyCord-to-ROS, 3. Quit): ")
        if c == 3: 
            break 
        elif c == 1: 
            xy = list(read_xy())                         
            cxy = ct.get_simple_xy_from_ros_xy(xy, sf)
            print cxy
        elif c == 2:
            xy = list(read_xy())
            rxy = ct.get_ros_xy_from_simple_xy(xy, sf)
            print rxy
        

if __name__ == '__main__':
    main()
    