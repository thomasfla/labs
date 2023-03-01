#Vulliez, Flayols, Apr 2022
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np
 
import pinocchio
from example_robot_data import *
import example_robot_data as erd
import numpy as np
from numpy.linalg import norm, solve, pinv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog 
np.set_printoptions(precision=3,linewidth=180)
 
from ik import *
    
    
robot    = TalosLoaderNoFF().robot
robot.initViewer(loadModel=True)
JOINT_ID = robot.model.getJointId("leg_right_6_joint")

# Simple mouse click function to store coordinates
def onclick(event):
    x,z = event.xdata, event.ydata
    if (x != None and z!= None):
        print(x,z)
        oMdes = pinocchio.SE3(np.eye(3), np.array([x, 0., z]))
        q = ik(robot,oMdes,JOINT_ID)
        robot.display(q)
    
data = np.load("fmax_talos.npz")
N=data['N']+0
F=data['F']
x=data['x']
z=data['z']
X, Z = np.meshgrid(x, z,indexing='ij')
# plot
names = ["Fx","Fy","Fz","Mx","My","Mz"]
vmaxs  = [2000,2000,2000,200,200,200]
fig, axs = plt.subplots(3,2)
i=0
for name in names:
    pcm = axs[i%3 , int(i/3)].pcolormesh(X, Z, F[i],cmap='viridis_r')
    axs[i%3 , int(i/3)].set_title(name)
    axs[i%3 , int(i/3)].axis('equal')
    fig.colorbar(pcm,ax=axs[i%3 , int(i/3)],shrink=0.8)
    i+=1
    
cid = fig.canvas.mpl_connect('motion_notify_event', onclick)

plt.show()

