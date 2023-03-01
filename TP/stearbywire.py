#!/usr/bin/python3
from param import offsets #Edit offset.py according to your testbench
from delta_utils import *
import time
from odri_spi_rpi import SPIuDriver
import time
dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=offsets)
ud.goto(0,0)
ud.transfer()
N=300000 #30 seconds
t = time.perf_counter()
i=0
p0 = np.array([0,0.1])
R = 0.02
for i in range(N):
    ud.refCurrent0 = 3 * (ud.position1 - ud.position0) + 0.03 * (ud.velocity1 - ud.velocity0)
    ud.refCurrent1 = 3 * (ud.position0 - ud.position1) + 0.03 * (ud.velocity0 - ud.velocity1)

    if ud.refCurrent0 > 2:
        ud.refCurrent0 = 2
    if ud.refCurrent0 <-2 :
        ud.refCurrent0 = -2

    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
ud.stop() #Terminate
