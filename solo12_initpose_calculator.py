#Tflayols LAAS
USE_VIEWER = True
import pinocchio as pin
import numpy as np
from IPython import embed
from example_robot_data import load

robot = load('solo12')
rmodel= robot.model
rdata= robot.data

if USE_VIEWER:
    robot.initViewer(loadModel=True)
    robot.viewer.gui.setRefreshIsSynchronous(False)

def invgeom(h = 0.2, lx = 0.1946, ly=0.16891, leg_dir = [+1,+1,-1,-1]):
    #Inputs to be modified by the user
    feet_position_ref =     [np.array([lx,   ly, 0.0191028]),np.array([lx,  -ly, 0.0191028]),np.array([-lx,   ly, 0.0191028]),np.array([-lx,  -ly, 0.0191028])]
    base_orientation_ref = pin.utils.rpyToMatrix(0,0,0) 
    com_ref = np.array([0,0,h])
    FL_FOOT_ID = robot.model.getFrameId('FL_FOOT')
    FR_FOOT_ID = robot.model.getFrameId('FR_FOOT')
    HL_FOOT_ID = robot.model.getFrameId('HL_FOOT')
    HR_FOOT_ID = robot.model.getFrameId('HR_FOOT')
    BASE_ID =  robot.model.getFrameId('base_link')
    foot_ids = np.array([FL_FOOT_ID, FR_FOOT_ID, HL_FOOT_ID, HR_FOOT_ID])
    q = np.array([ 0., 0.,0.235,  0.   ,  0.   ,  0.   ,  1.   ,  
                0.1  , +0.8 * leg_dir[0] , -1.6 * leg_dir[0] , 
               -0.1  , +0.8 * leg_dir[1],  -1.6 * leg_dir[1] ,  
                0.1  , -0.8 * leg_dir[2] , +1.6 * leg_dir[2] ,
               -0.1  , -0.8 * leg_dir[3] , +1.6 * leg_dir[3]])
    dq = robot.v0.copy()
    for i in range(100):
        #Compute fwd kin
        pin.forwardKinematics(rmodel,rdata,q,np.zeros(rmodel.nv),np.zeros(rmodel.nv))
        pin.updateFramePlacements(rmodel,rdata)

        ### FEET
        Jfeet = []
        pfeet_err = []
        for i_ee in range(4):
            idx = int(foot_ids[i_ee])
            pos = rdata.oMf[idx].translation
            ref = feet_position_ref[i_ee]
            Jfeet.append(pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3])
            pfeet_err.append(ref-pos)

        ### CoM
        com = robot.com(q)
        Jcom = robot.Jcom(q)
        e_com = com_ref-com

        ### BASE ROTATION
        idx = BASE_ID
        rot = rdata.oMf[idx].rotation
        rotref = base_orientation_ref
        Jwbasis = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[3:]
        e_basisrot = -rotref @ pin.log3(rotref.T@rot)

        #All tasks
        J = np.vstack(Jfeet+[Jcom,Jwbasis])
        x_err = np.concatenate(pfeet_err+[e_com,e_basisrot])
        residual = np.linalg.norm(x_err)
        if residual<1e-5:
            print (np.linalg.norm(x_err))
            print ("iteration: {} residual: {}".format(i,residual))
            return q
        dq=np.linalg.pinv(J)@(x_err)
        q=pin.integrate(robot.model,q,dq)
    return q * np.nan

from tkinter import *
def update_callback(arg=None):
    h  = w_h.get()*1e-3
    lx = w_lx.get()*1e-3
    ly = w_ly.get()*1e-3
    q = invgeom(h, lx, ly, leg_dir = [ld1.get(),ld2.get(),ld3.get(),ld4.get()])
    robot.display(q)
    print (q)

master = Tk()
ld1 = IntVar()
ld2 = IntVar()
ld3 = IntVar()
ld4 = IntVar()

w_h = Scale(master, from_=0, to=400, tickinterval=1, orient=HORIZONTAL,command=update_callback)
w_h.set(200)
w_h.pack()

w_lx = Scale(master, from_=0, to=300,tickinterval=1, orient=HORIZONTAL,command=update_callback)
w_lx.set(194)
w_lx.pack()

w_ly = Scale(master, from_=0, to=200,tickinterval=1, orient=HORIZONTAL, command=update_callback)
w_ly.set(169)
w_ly.pack()

c1 = Checkbutton(text='LEG0 direction', var=ld1, onvalue=1, offvalue=-1, command=update_callback)
c1.pack()
c2 = Checkbutton(text='LEG0 direction', var=ld2, onvalue=1, offvalue=-1, command=update_callback)
c2.pack()
c3 = Checkbutton(text='LEG0 direction', var=ld3, onvalue=1, offvalue=-1, command=update_callback)
c3.pack()
c4 = Checkbutton(text='LEG0 direction', var=ld4, onvalue=1, offvalue=-1, command=update_callback)
c4.pack()

mainloop()

    
