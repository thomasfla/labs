#!/usr/bin/python3
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
ud = SPIuDriver(absolutePositionMode=True, offsets=[-0.27,-0.12])
ud.transfer()
ud.goto(pi/2,pi/2)
t = time.perf_counter()
i=0
while True:
    i+=1
    q = np.array([ud.position0,ud.position1])
    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
    if (i%100==0):
        #print(f"q={q}")
        print(f"fk_delta(q)={fk_delta(q)}")

ud.stop() #Terminate