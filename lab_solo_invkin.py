USE_VIEWER = True
import pinocchio as pin
import numpy as np
import time
from IPython import embed
import time
from example_robot_data import load
robot = load('solo12')

#Inputs
dt = 0.001
feet_position_ref =     [0*np.array([0.1946,   0.16891, 0.0191028]),0.0*np.array([0.1946,  -0.16891, 0.0191028]),np.array([-0.1946,   0.16891, 0.0191028]),np.array([-0.1946,  -0.16891, 0.0191028])]
feet_velocity_ref =     [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
feet_acceleration_ref = [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
flag_in_contact = np.array([0,0,0,1])
base_orientation_ref = pin.utils.rpyToMatrix(0,0,np.pi) #BUG?? Why doesn't it converge to the ref? 
base_angularvelocity_ref = np.array([0,0,0])
base_angularacceleration_ref = np.array([0,0,0])
base_position_ref = np.array([0,0,0.235])
base_linearvelocity_ref = np.array([0,0,0])
base_linearacceleration_ref = np.array([0,0,0])

Kp_base_orientation = 1000
Kd_base_orientation = 2*np.sqrt(Kp_base_orientation)

Kp_base_position = 100
Kd_base_position = 2*np.sqrt(Kp_base_position)

Kp_flyingfeet = 100
Kd_flyingfeet = 2*np.sqrt(Kp_flyingfeet)

#pin.switchToNumpyMatrix()
if USE_VIEWER:
    robot.initViewer(loadModel=True)
    robot.viewer.gui.setRefreshIsSynchronous(False)
    robot.display(robot.q0)

q = robot.q0.copy()
dq = robot.v0.copy()
ddq = dq *0
#Get frame IDs
FL_FOOT_ID = robot.model.getFrameId('FL_FOOT')
FR_FOOT_ID = robot.model.getFrameId('FR_FOOT')
HL_FOOT_ID = robot.model.getFrameId('HL_FOOT')
HR_FOOT_ID = robot.model.getFrameId('HR_FOOT')
BASE_ID = robot.model.getFrameId('base_link')



foot_ids = np.array([FL_FOOT_ID, FR_FOOT_ID, HL_FOOT_ID, HR_FOOT_ID])

for i in range (2000):
    robot.forwardKinematics(q,dq,ddq) #include add ddq?
    J_FL_foot = robot.computeFrameJacobian(q,FL_FOOT_ID)[:3]
    J_FR_foot = robot.computeFrameJacobian(q,FR_FOOT_ID)[:3]
    J_HL_foot = robot.computeFrameJacobian(q,HL_FOOT_ID)[:3]
    J_HR_foot = robot.computeFrameJacobian(q,HR_FOOT_ID)[:3]

    #creating contact jacobian / references
    Jc = np.empty([3*flag_in_contact.sum(),18])*np.nan
    contacts_pos = np.empty(3*flag_in_contact.sum())*np.nan
    n=0
    for i_ee in  range(4) :
        if flag_in_contact[i_ee]:
            Jcontact = robot.computeFrameJacobian(q,int(foot_ids[i_ee]))[:3]
            Jc[n*3:(n+1)*3] = Jcontact
            contacts_pos[n*3:(n+1)*3] = feet_position_ref[i_ee] #not used..
            n=n+1

    #creating flying foot tracking jacobians / references
    J_flyingfeet = np.empty([3*(4-flag_in_contact.sum()),18])*np.nan
    flyingfeet_pos_ref = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    flyingfeet_vel_ref = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    flyingfeet_acc_ref = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    flyingfeet_pos = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    flyingfeet_vel = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    dJdq_flyingfeet = np.empty(3*(4-flag_in_contact.sum()))*np.nan
    n=0
    for i_ee in  range(4) :
        if (not flag_in_contact[i_ee]):
            id = int(foot_ids[i_ee])
            J_flyingfoot = robot.computeFrameJacobian(q,id)[:3]
            dJdq_flyingfeet[n*3:(n+1)*3] = 0#robot.frameAcceleration(q,dq,0*dq,id).linear #todo check
            J_flyingfeet[n*3:(n+1)*3] = J_flyingfoot
            flyingfeet_pos[n*3:(n+1)*3] = robot.framePlacement(q,id).translation
            flyingfeet_vel[n*3:(n+1)*3] = robot.frameVelocity(q,dq,id).linear
            flyingfeet_pos_ref[n*3:(n+1)*3] = feet_position_ref[i_ee]
            flyingfeet_vel_ref[n*3:(n+1)*3] = feet_velocity_ref[i_ee]
            flyingfeet_acc_ref[n*3:(n+1)*3] = feet_acceleration_ref[i_ee]
            n=n+1

    #base jacobians
    J_base = robot.computeFrameJacobian(q,BASE_ID)
    J_base_position = J_base[:3]
    J_base_orientation = J_base[3:]

    pinv = np.linalg.pinv
    inv = np.linalg.inv

    #WBIC from https://arxiv.org/pdf/1909.06586.pdf
    M = robot.mass(q)

    #base orientation (3d)
    Kp1 = Kp_base_orientation
    Kd1 = Kd_base_orientation
    J1 = J_base_orientation
    M1 = robot.framePlacement(q,BASE_ID).rotation

    dx1 = robot.frameVelocity(q,dq,BASE_ID).angular
    M1_des = base_orientation_ref
    dx1_des = base_angularvelocity_ref
    ddx1_des = base_angularacceleration_ref
    e1 = pin.log(M1_des.T*M1)
    ddx1_cmd = ddx1_des + Kp1 * e1 + Kd1 * (dx1_des - dx1)
    dJ1dq = 0#robot.frameAcceleration(q,dq,0*dq,BASE_ID).angular # TODO check this !

    #base position (3d)
    Kp2 = Kp_base_position
    Kd2 = Kd_base_position
    J2 = J_base_position
    x2 = robot.framePlacement(q,BASE_ID).translation
    dx2 = robot.frameVelocity(q,dq,BASE_ID).linear
    x2_des = base_position_ref
    dx2_des = base_linearvelocity_ref
    ddx2_des = base_linearacceleration_ref
    e2 = x2_des-x2
    ddx2_cmd = ddx2_des + Kp2 * e2 + Kd2 * (dx2_des - dx2)
    dJ2dq = 0#robot.frameAcceleration(q,dq,0*dq,BASE_ID).linear # TODO check this !

    #feet position
    Kp3 = Kp_flyingfeet
    Kd3 = Kd_flyingfeet
    J3 = J_flyingfeet
    x3 = flyingfeet_pos
    dx3 = flyingfeet_vel
    x3_des = flyingfeet_pos_ref
    dx3_des = flyingfeet_vel_ref
    ddx3_des = flyingfeet_acc_ref
    e3 = x3_des-x3
    ddx3_cmd = ddx3_des + Kp3 * e3 + Kd3 * (dx3_des - dx3)
    dJ3dq = dJdq_flyingfeet
    # null spaces and projections
    N0 = np.eye(robot.nv) - pinv(Jc)@Jc
    J1_pre = J1 @ N0

    N10 = np.eye(robot.nv) - pinv(J1_pre)@J1_pre
    N1 = N0 @ N10
    J2_pre = J2 @ N1

    N21 = np.eye(robot.nv) - pinv(J2_pre)@J2_pre
    N2 = N0 @ N10 @ N21
    J3_pre = J3 @ N2


    def dynPinv(M,J):
        invM = np.linalg.inv(M)
        return ( invM  @ J.T @ np.linalg.inv(J @ invM @ J.T)) #todo optimize?

    delta_q0 = np.zeros(18)
    delta_q1 = delta_q0 + pinv(J1_pre) @ (e1 - J1 @ delta_q0)
    delta_q2 = delta_q1 + pinv(J2_pre) @ (e2 - J2 @ delta_q1)
    delta_q3 = delta_q2 + pinv(J3_pre) @ (e3 - J3 @ delta_q2)

    dq0_cmd = np.zeros(18)
    dq1_cmd = dq0_cmd + pinv(J1_pre) @ (dx1_des - J1 @ dq0_cmd)
    dq2_cmd = dq1_cmd + pinv(J2_pre) @ (dx2_des - J2 @ dq1_cmd)
    dq3_cmd = dq2_cmd + pinv(J3_pre) @ (dx3_des - J3 @ dq2_cmd)

    # todo use dynPinv for accelerations ...
    ddq0_cmd = pinv(Jc) @ (-Jc@dq)
    ddq1_cmd = ddq0_cmd + pinv(J1_pre) @ (ddx1_cmd - dJ1dq - J1 @ ddq0_cmd) 
    ddq2_cmd = ddq1_cmd + pinv(J2_pre) @ (ddx2_cmd - dJ2dq - J2 @ ddq1_cmd)
    ddq3_cmd = ddq2_cmd + pinv(J3_pre) @ (ddx3_cmd - dJ3dq - J3 @ ddq2_cmd)

    #print ("e1", e1)
    #print ("e2", e2)
    #print ("e3", e3)
    #pure integration
    #time.sleep(0.1)
    ddq=ddq3_cmd
    #print (ddq)
    dq=dq+dt*ddq
    q=pin.integrate(robot.model,q,dq*dt)
    if USE_VIEWER:
        if (i%10==0):
            print (i)
            robot.display(q)
embed()
    
print ("end of script")
