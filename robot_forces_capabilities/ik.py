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
np.set_printoptions(precision=3,linewidth=180)
 
class TalosLoaderNoFF(erd.robots_loader.RobotLoader):
    path = "talos_data"
    urdf_filename = "talos_reduced.urdf"
    srdf_filename = "talos.srdf"
    free_flyer = False
    has_rotor_parameters = True


def ik(robot,oMdes, JOINT_ID,eps = 1e-4):
    q = robot.q0.copy()
    IT_MAX = 500
    DT     = 1e-1
    damp   = 1e-12
    i=0
    success = False
    while True:
        pinocchio.forwardKinematics(robot.model,robot.data,q)
        dMi = oMdes.actInv(robot.data.oMi[JOINT_ID])
        err = pinocchio.log(dMi).vector
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        J = pinocchio.computeJointJacobian(robot.model,robot.data,q,JOINT_ID)
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(robot.model,q,v*DT)
        i += 1
    if not success:
        #raise( Exception('ik did not converged') )
        q[:] = np.nan
    if ((q < robot.model.lowerPositionLimit).any() or
    (q > robot.model.upperPositionLimit).any()):
        #Violate joint limits
        q[:] = np.nan
    return (q)
