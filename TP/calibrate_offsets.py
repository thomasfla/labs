#!/usr/bin/python3
from delta_utils import *
import time
from odri_spi_rpi import SPIuDriver
import time
dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=[0,0])
ud.goto(0,0) 
ud.goto(0,0) # To hold the motor steady
ud.transfer()
N=30000 #30 seconds
t = time.perf_counter()
for i in range(N):
    ud.refCurrent0 = 0;
    ud.refCurrent1 = 0;
    if (i%10==0):
        offset0 = ((ud.position0+pi)%(2*pi))-pi # result in [-pi ; pi]
        offset1 = ((ud.position1+pi)%(2*pi))-pi 
        print(f"offsets=[{-offset0},{-offset1}]")
    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
ud.stop() #Terminate
