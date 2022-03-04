#!/usr/bin/python3
from param import offsets #Edit offset.py according to your testbench
from delta_utils import *
import time
from odri_spi_rpi import *
import time
dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=offsets)
ud.transfer()
ud.goto(pi/2,pi/2)
ud.goto(pi/2,pi/2)
N=300000 #30 seconds
t = time.perf_counter()
i=0
p0 = np.array([0,0.1])
R = 0.02
for i in range(N):
    q = np.array([ud.position0,ud.position1])
    p = fk_delta(q)
    d = np.linalg.norm(p-p0)
    f = np.array([0.,0.])
    if (d<R):
        f = (p-p0)/d
    tau = J(q).T @ f
    ud.refCurrent0 = tau[0] * 30
    ud.refCurrent1 = tau[1] * 30
    
    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
    if (i%100==0):
        #print(f"q={q}")
        #print(f"p=fk_delta(q)={p}")
        print(f"f={f}, tau={tau}")

ud.stop() #Terminate
