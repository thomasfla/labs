#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Thomas Flayols, feb 2022

from math import atan, acos, cos, sin, atan2
import matplotlib.pyplot as plt
from IPython import embed

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

def ik_delta(x,y):
    q1,_ = ik_serial(x-d,y,True)
    q2,_ = ik_serial(x+d,y,False)
    return (q1,q2)
    
def fk_delta():
    return

def J(q):
    eps = 1e-3
    #todo
    return 
    
def plot_limits():
    plt.plot([-0.105,0.105,0.105,-0.105,-0.105],[0.04,0.04,-0.04,-0.04,0.04])
    
    
x,y = -0.00,0.165

plot_limits()
q1,q2 = ik_serial(x-d,y,True)
plot_serial(q1,q2,d)
q1,q2 = ik_serial(x+d,y,False)
plot_serial(q1,q2,-d)
print(ik_delta(x,y))
embed()
plt.show()
