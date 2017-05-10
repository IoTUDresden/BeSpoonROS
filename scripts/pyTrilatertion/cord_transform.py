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

    return set([Rx, Ry])

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

if __name__ == '__main__':
    p = [[20, 80], [450, 45], [300, 580]]
    # p = [20,80]
    # b = [0.391069, 2.99875]
    # c = [ x/100.0  for x in p]    

    # B's center in R's plane     
    b = [-0.191069, 3.79875]
    angle = 270.0
    
    # 1m = 100cm, use same metric for both coordinates
    pp = [ [ y/100.0 for y in x] for x in p]
    for x in pp: print x, transform(x, angle, b)  
    # print transform(c,270.0, b)
    pass