#!/usr/bin/python3
from param import offsets #Edit offset.py according to your testbench
from delta_utils import *
import time
from IPython import embed
from odri_spi_rpi import *
import time
dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=offsets)
ud.transfer()
ud.goto(0,0)
ud.goto(0,0)
N=30000 #30 seconds
t = time.perf_counter()
i=0
for i in range(N):
    q = np.array([ud.position0,ud.position1])
    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
    if (i%100==0):
        print(f"q={q}")

ud.stop() #Terminate