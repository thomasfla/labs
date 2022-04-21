#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Thomas Flayols, feb 2022

from math import atan, acos, cos, sin, atan2, sqrt
import matplotlib.pyplot as plt
from IPython import embed
from numpy.linalg import inv
import numpy as np
from numpy import pi

#System size          #        /\x,y
l1 = 0.06             #       /  \
l2 = 0.125            #  ^   /    \<-l2
d = 0.130 / 2         #  |   \_dd_/<-l1
                      #  |   q1  q2
                      # Y|
                      #  |____>
                      #    X
                                       
def ik_serial(x,y, positive=True):
    sign = 1 if positive else -1
    l12=l1*l1
    l22=l2*l2
    x2=x*x
    y2=y*y
    q2 = sign * acos( (x2+y2-l12-l22) / (2*l1*l2) )
    q1 = atan2( y , x ) - atan2((l2 * sin(q2)) , (l1 + l2*(cos(q2))))
    return (q1,q2)

def plot_serial(q1,q2,x0=0.0):
    plt.axis('equal')
    xa = cos(q1) * l1 + x0
    ya = sin(q1) * l1
    xb = xa + cos(q1+q2) * l2
    yb = ya + sin(q1+q2) * l2
    plt.plot([x0,xa,xb] , [0,ya,yb],"o-")

def ik_delta(p):
    x = p[0]
    y = p[1]
    q1,_ = ik_serial(x-d,y,True)
    q0,_ = ik_serial(x+d,y,False)
    return np.array([q0,q1])
    
def plot_delta(q,line="o-"):
    plt.axis('equal')
    x,y = fk_delta(q)

    xl = cos(q[0]) * l1 - d
    yl = sin(q[0]) * l1
    
    xr = cos(q[1]) * l1 + d
    yr = sin(q[1]) * l1
    plt.plot([-d,xl,x] , [0,yl,y],line,color="red")
    plt.plot([+d,xr,x] , [0,yr,y],line,color="blue")

def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    d=sqrt((x1-x0)**2 + (y1-y0)**2)
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return np.array([np.nan,np.nan])
    # coincident circles
    if d == 0 and r0 == r1:
        return np.array([np.nan,np.nan])
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 
        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        return (x3, y3, x4, y4)

def fk_delta(q,positive=True):
    sign = 1 if positive else -1
    x0 = -d + l1*cos(q[0])
    y0 =      l1*sin(q[0])
    x1 = d + l1*cos(q[1])
    y1 =     l1*sin(q[1])
    (x3, y3, x4, y4) = get_intersections(x0,y0,l2,x1,y1,l2)
    return (np.array([x4,y4]))

def J(q):
    eps = 1e-6
    dq0 = eps * np.array([1,0]) 
    dq1 = eps * np.array([0,1])

    return 1./eps * np.array([fk_delta(q+dq0) - fk_delta(q),fk_delta(q+dq1) - fk_delta(q)]).T
    
def plot_box():
    plt.plot([-0.105,0.105,0.105,-0.105,-0.105],[0.04,0.04,-0.04,-0.04,0.04])
    
    

