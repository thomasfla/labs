#!/usr/bin/python3
from param import offsets #Edit offset.py according to your testbench
from delta_utils import *
p = np.array([0,0.1])
q = ik_delta(p)
print (f" p = {p}")
print (f" q = IK(p) = {q}")
import time
from IPython import embed
from odri_spi_rpi import *
import time
dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=offsets)
ud.transfer()
ud.goto(pi/2,pi/2)
ud.goto(pi/2,pi/2)
N=30000 #30 seconds
t = time.perf_counter()
ps = np.empty([N,2])
i=0
for i in range(N):
    q = np.array([ud.position0,ud.position1])
    p = fk_delta(q)
    ps[i]=p
    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
    if (i%100==0):
        #print(f"q={q}")
        print(f"p=fk_delta(q)={p}")

ud.stop() #Terminate
plt.axis('equal')
plt.plot(ps[:,0],ps[:,1])
plt.show()