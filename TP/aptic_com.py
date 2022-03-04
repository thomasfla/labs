#!/usr/bin/python3
import time
from IPython import embed
from odri_spi_rpi import *
import time
import struct

from param import offsets #Edit offset.py according to your testbench
import numpy as np
import matplotlib.pyplot as plt

from delta_utils import *

import socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('', 12000))
server_socket.setblocking(False)

send_socket = socket.socket(type=socket.SOCK_DGRAM)

dt = 0.001
ud = SPIuDriver(absolutePositionMode=True, offsets=offsets)
ud.transfer()
ud.goto(0,0)
q = np.array([0., 0.])
v = np.array([0., 0.])

ud.kp0= 5.
ud.kp1= 5.
ud.kd0= 0.2
ud.kd1= 0.2

t = time.perf_counter()
i=0
while True:
    try:
        message, address = server_socket.recvfrom(1024)
        s = struct.unpack('ff', message)
        q = np.array(s)
    except BlockingIOError:
        pass
    if i==10:
        send_socket.sendto(struct.pack('ff', ud.position0, ud.position1), ('140.93.16.44',12000))
        i=0
    i = i + 1
    ud.refPosition0 = q[0]
    ud.refPosition1 = q[1]
    ud.refVelocity0 = v[0]
    ud.refVelocity1 = v[1]

    ud.transfer() #transfer
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)

ud.stop() #Terminate