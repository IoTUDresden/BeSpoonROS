#!/usr/bin/env python 
import math
import numpy

"""
Trilateration in XY plane

https://en.wikipedia.org/wiki/Trilateration
"""

def trilateration(points):
    if not isinstance(points, list): return ()

    P1 = numpy.array(points[0][:3])
    P2 = numpy.array(points[1][:3])
    P3 = numpy.array(points[2][:3])

    d1 = points[0][3]
    d2 = points[1][3]
    d3 = points[2][3]

    #transform to get circle 1 at origin
    #transform to get circle 2 on x axis
    ex = (P2 - P1)/(numpy.linalg.norm(P2 - P1))
    i = numpy.dot(ex, P3 - P1)
    ey = (P3 - P1 - i*ex)/(numpy.linalg.norm(P3 - P1 - i*ex))
    ez = numpy.cross(ex,ey)
    d = numpy.linalg.norm(P2 - P1)
    j = numpy.dot(ey, P3 - P1)

    x = (pow(d1,2) - pow(d2,2) + pow(d,2))/(2*d)
    y = ((pow(d1,2) - pow(d3,2) + pow(i,2) + pow(j,2))/(2*j)) - ((i/j)*x)        
    z = 0 # numpy.sqrt(pow(d1,2) - pow(x,2) - pow(y,2))    
    P12 = P1 + x*ex + y*ey + z*ez
    #print ex, ey, ez, i, j, d, P12
    print d
    print 'P12:', list(P12)
    return (x,y,z)


def filter():
    pass     


if __name__ == '__main__':
    #  p1, p2, p3:  [x, y, z, d]
    data = [ [-0.210776084205, 0.0606242891579, 0, 0.219321], 
    [4.25212874019 , 0.468160024585, 0, 4.277823], 
    [0.391068983923, 2.99875148891, 0, 3.024144] ]

    data = [ [-0.210776084205, 0.0606242891579, 0, 0.219321], 
    [4.25212874019 , 0.468160024585, 0, 4.277823], 
    [0.391068983923, 2.99875148891, 0, 3.024144] ]

    # print(data)
    val = trilateration(data)    
    print(val)
    




