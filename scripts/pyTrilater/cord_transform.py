#!/usr/bin/env python 

import math 
import numpy 

"""
Two different XY plane B (BeSpoon), R (ROS). Point T(x,y) in B has to be mapped
in R plane. 
For simplicity: Z=0 

Pesudo: 
1. Determine R coordinates of B's origin, R(h,k)    
    1.1. Find the B(0,0) in R, let assume R_bcenter=(h, k) 
    1.2. C(x, y) = R(0,0) - R_bcenter=(h, k)
2. Parallelize the both planes in respective with X-axis (translation axis) of B & R
    2.1. Find the anticlockwise angle (d) of X-axis of B in R
    2.2. If parrallel: nothing to do 
    2.3. else: rotate B-plane by d-degree 
3. Translate T(x,y) to R(x',y'):
    3.1. matrix:  [[Rx'],[Ry']] = ([[cos(d), sin(d)][-sin(d), cos(d)]]) dot ([[Tx],[Ty]] - [[h],[k]])
        or 
        Rx' = (Tx - h) cos(d) + (Ty - k) sin(d)
        Ry' = -(Tx - h) sin(d) + (Ty - k) cos(d)
"""

def transform(point, angle_in_degree=None, point_origin_in_R=None):
    """
    transform BeSpoon XYZ cordinate to ROS frame cordinate.
    Measurement base unit is 'm' (meter). 
    Z=0 for in both plane 
    """
    if angle_in_degree is None: angle_in_degree = 0.0
    #  base is the coefficient of ROS frame (0,0,0), default is [0,0,0]
    if point_origin_in_R is None: base = [0,0] 
    else: base = point_origin_in_R
    
    ## In radian         
    # angle =  (3*math.pi)/2     
    angle = math.radians(angle_in_degree)   
    
    Rx = (point[0] - base[0]) * math.cos(angle)  + (point[1] - base[1]) * math.sin(angle)
    Ry =  -1 * (point[0] - base[0]) * math.sin(angle)  + (point[1] - base[1]) * math.cos(angle)
    # TODO z, z = 0     
    return [Rx, Ry, 0.0]

def get_angle(Bpoint, Rcenter=None):
    """
    d = deltaY / deltaX # (y2-y1)/(x2-x1)
    """
    if Rcenter is None: Rcenter = [0,0]    
    deltaX = Bpoint[0] - Rcenter[0]
    deltaY = Bpoint[1] - Rcenter[1]
    radian = math.atan2(deltaY, deltaX)
    angle = radian
    # angle = math.degrees(radian)    
    return angle

def get_plane_angle(bCord, rCord):
    """
    cos(d) = |A1*A2 + B1*B2| / (sqrt(A1^2+B1^2)*sqrt(A2^2+B2^2))
    """
    ab = abs(bCord[0] * rCord[0] + bCord[1]*rCord[1])
    abr = (math.sqrt( bCord[0]**2 + bCord[1]**2) * math.sqrt( rCord[0]**2 + rCord[1]**2))
    angle = math.degrees(math.acos(ab/abr)) 
    return angle
    
def get_euclidean_distance(p1, p2):
    a = numpy.array(p1)
    b = numpy.array(p2)
    dist = numpy.linalg.norm(a-b)
    return dist 

def get_distance(p1=None, p2=None):
    if p1 is None or p2 is None: return None 
    if len(p1) < 3 or len(p2) < 3: return None 
    
    dist = math.sqrt(  pow((p1[0]-p2[0]),2) + pow((p1[1]-p2[1]),2) + pow((p1[2]-p2[2]),2) )
    return dist    

def get_ros_xy(point): 
    if not isinstance(point, list) and len(point) < 2: 
        raise Exception("Invalid coordinate, expected: [x,y]")
    
    rp = [ x * ros_axis_scale_factor for x in point]        
    ros_xy = transform(rp, bespoon_plane_angle_with_ros_plane, bespoon_center_in_ros_plane)
    return ros_xy 

def get_simple_xy_from_ros_xy(ros_coord, scale_factor=None):
    """
    Get simple xy coordinate from original ros xy coordinate, based on scale factor, angle and ros-center 
    """
    if not isinstance(ros_coord, list) and len(ros_coord) < 2: 
        raise Exception("Invalid coordinate, expected: [x,y]")    

    if scale_factor is None: 
        scale_factor = simple_xy_scale_factor
    elif isinstance(scale_factor, (int, float)) is not True: 
        raise Exception('Invalid scale_factor, expected: int or float')

    cp = [ x / float(scale_factor) for x in ros_coord]   
    # plane angle and center remain constant 
    my_xy = transform(cp, bespoon_plane_angle_with_ros_plane, bespoon_center_in_ros_plane)
    return my_xy

def get_ros_xy_from_simple_xy(my_coord, scale_factor=None):
    """
    Get ros xy from simple xy coordinate based on scale factor, angle and ros-center 
    """
    if not isinstance(my_coord, list) and len(my_coord) < 2: 
        raise Exception("Invalid coordinate, expected: [x,y]")    

    if scale_factor is None: 
        scale_factor = simple_xy_scale_factor
    elif isinstance(scale_factor, (int, float)) is not True: 
        raise Exception('Invalid scale_factor, expected: int or float')

    rp = [ x * float(scale_factor) for x in my_coord]   
    # plane angle and center remain constant 
    ros_xy = transform(rp, bespoon_plane_angle_with_ros_plane, bespoon_center_in_ros_plane)
    return ros_xy

# Ros axis in meter but bespoon axis in cm, 
# 1m=100cm, unit_factor = 1/100 = 0.01 
ros_axis_scale_factor = 0.01
# currently, x-axis parallel, hence angle=0.0 
bespoon_plane_angle_with_ros_plane = 0.0
# bespoon plane's (0,0) coordinate in R plane, default [0,0]
bespoon_center_in_ros_plane = [0,0]
# Simple XY grid scale factor 
# 1 unit = 20 cm; conversion factor = 20cm/1m = 20cm/100cm = 0.20
simple_xy_scale_factor = 0.20 

if __name__ == '__main__':
    p = [[20, 80], [450, 45], [300, 580]]
    
    # ToDo: update variable if needed 
    bespoon_plane_angle_with_ros_plane = 0
    bespoon_center_in_ros_plane = [0,0]
    # ros_axis_scale_factor = 0.20 

    for x in p:
        rxy = get_ros_xy(x)      
        cxy = get_simple_xy_from_ros_xy(rxy)
        rr = get_ros_xy_from_simple_xy(cxy)
        print x, rxy, cxy, cmp(rxy, rr) 