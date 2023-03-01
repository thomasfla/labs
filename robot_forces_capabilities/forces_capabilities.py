#Vulliez, Flayols, Apr 2022
import pinocchio
from IPython import embed
from example_robot_data import *
import example_robot_data as erd
import numpy as np
from numpy.linalg import norm, solve, pinv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog 

from ik import *
np.set_printoptions(precision=3,linewidth=180)


robot    = TalosLoaderNoFF().robot
robot.initViewer(loadModel=True)
JOINT_ID = robot.model.getJointId("leg_right_6_joint")
tau_max = robot.model.effortLimit

q = robot.q0


#Compute max contact forces on the X-Z plan
#plt.style.use('_mpl-gallery-nogrid')

# make data with uneven sampling in x
N = 100
x = np.linspace(-0.65,0.65,N)
z = np.linspace(-1.0,-0.4,N)
F = np.empty([6,N,N])
for ix in range(N):
    print(f"{ix/N *100} %")
    for iz in range(N):
        oMdes = pinocchio.SE3(np.eye(3), np.array([x[ix], 0., z[iz]]))
        q = ik(robot,oMdes,JOINT_ID)
        f_max = np.ones(6)*np.nan
        if not np.isnan(q).any():
            J = pinocchio.computeJointJacobian(robot.model,robot.data,q,JOINT_ID)
            for axis in range(6):
                c = np.array([0.,0.,0.,0.,0.,0.])
                c[axis]=-1.0
                A_ub = np.vstack([J.T,-J.T])
                b_ub = np.concatenate([tau_max,tau_max])
                f_max[axis] = linprog(c, A_ub=A_ub, b_ub=b_ub).x[axis]            
            #robot.display(q)
        F[:,ix,iz] = f_max
np.savez("fmax_talos.npz",F=F,x=x,z=z,N=N)




embed()
