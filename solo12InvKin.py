
USE_VIEWER = True
import pinocchio as pin
import numpy as np
import time
from IPython import embed
import time
from example_robot_data import load

class Solo12InvKin:
    def __init__(self):
        self.robot = load('solo12')

        #Inputs to be modified bu the user before calling .compute
        self.feet_position_ref =     [np.array([0.1946,   0.16891, 0.0191028]),np.array([0.1946,  -0.16891, 0.0191028]),np.array([-0.1946,   0.16891, 0.0191028]),np.array([-0.1946,  -0.16891, 0.0191028])]
        self.feet_velocity_ref =     [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
        self.feet_acceleration_ref = [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
        self.flag_in_contact = np.array([0,1,0,1])
        self.base_orientation_ref = pin.utils.rpyToMatrix(0,0,np.pi/6) 
        self.base_angularvelocity_ref = np.array([0,0,0])
        self.base_angularacceleration_ref = np.array([0,0,0])
        self.base_position_ref = np.array([0,0,0.235])
        self.base_linearvelocity_ref = np.array([0,0,0])
        self.base_linearacceleration_ref = np.array([0,0,0])

        self.Kp_base_orientation = 100
        self.Kd_base_orientation = 2*np.sqrt(self.Kp_base_orientation)

        self.Kp_base_position = 100
        self.Kd_base_position = 2*np.sqrt(self.Kp_base_position)

        self.Kp_flyingfeet = 100
        self.Kd_flyingfeet = 2*np.sqrt(self.Kp_flyingfeet)

        #Get frame IDs
        FL_FOOT_ID = self.robot.model.getFrameId('FL_FOOT')
        FR_FOOT_ID = self.robot.model.getFrameId('FR_FOOT')
        HL_FOOT_ID = self.robot.model.getFrameId('HL_FOOT')
        HR_FOOT_ID = self.robot.model.getFrameId('HR_FOOT')
        self.BASE_ID =  self.robot.model.getFrameId('base_link')
        self.foot_ids = np.array([FL_FOOT_ID, FR_FOOT_ID, HL_FOOT_ID, HR_FOOT_ID])
        def dinv(J,damping=1e-2):
            ''' Damped inverse '''
            U,S,V = np.linalg.svd(J)
            if damping==0:
                Sinv = 1/S
            else:
                Sinv = S/(S**2+damping**2)
            return (V.T*Sinv)@U.T

        self.rmodel= self.robot.model
        self.rdata= self.robot.data
        self.i=0

    def dinv(self,J,damping=1e-2):
        ''' Damped inverse '''
        U,S,V = np.linalg.svd(J)
        if damping==0:
            Sinv = 1/S
        else:
            Sinv = S/(S**2+damping**2)
        return (V.T*Sinv)@U.T


    def cross3(self, left, right):
        """Numpy is inefficient for this"""
        return np.array([left[1] * right[2] - left[2] * right[1],
                         left[2] * right[0] - left[0] * right[2],
                         left[0] * right[1] - left[1] * right[0]])

    def compute(self,q,dq):
        ### FEET
        Jfeet = []
        afeet = []
        pfeet_err = []
        vfeet_ref = []
        pin.forwardKinematics(self.rmodel,self.rdata,q,dq,np.zeros(self.rmodel.nv))
        pin.updateFramePlacements(self.rmodel,self.rdata)

        for i_ee in range(4):
            idx = int(self.foot_ids[i_ee])
            pos = self.rdata.oMf[idx].translation
            nu = pin.getFrameVelocity(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED)
            ref = self.feet_position_ref[i_ee]
            vref = self.feet_velocity_ref[i_ee]
            aref = self.feet_acceleration_ref[i_ee]
            
            J1 = pin.computeFrameJacobian(self.robot.model,self.robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
            e1 = ref-pos
            acc1 = -self.Kp_flyingfeet*(pos-ref) - self.Kd_flyingfeet*(nu.linear-vref) + aref
            if self.flag_in_contact[i_ee] :
                acc1 *= 0 # In contact = no feedback
            drift1 = np.zeros(3)
            drift1 += pin.getFrameAcceleration(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
            drift1 += self.cross3(nu.angular,nu.linear)
            acc1 -= drift1

            Jfeet.append(J1)
            afeet.append(acc1)

            pfeet_err.append(e1)
            vfeet_ref.append(vref)

        ### BASE POSITION
        idx = self.BASE_ID
        pos = self.rdata.oMf[idx].translation
        nu = pin.getFrameVelocity(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED)
        ref = self.base_position_ref
        Jbasis = pin.computeFrameJacobian(self.robot.model,self.robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
        e_basispos = ref - pos
        accbasis = -self.Kp_base_position*(pos-ref) - self.Kd_base_position*nu.linear
        drift = np.zeros(3)
        drift += pin.getFrameAcceleration(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
        drift += self.cross3(nu.angular,nu.linear)
        accbasis -= drift

        ### BASE ROTATION
        idx = self.BASE_ID

        rot = self.rdata.oMf[idx].rotation
        nu = pin.getFrameVelocity(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED)
        rotref = self.base_orientation_ref
        Jwbasis = pin.computeFrameJacobian(self.robot.model,self.robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[3:]
        e_basisrot = -rotref @ pin.log3(rotref.T@rot)
        accwbasis = -self.Kp_base_orientation * rotref @ pin.log3(rotref.T@rot) - self.Kd_base_orientation*nu.angular
        drift = np.zeros(3)
        drift += pin.getFrameAcceleration(self.rmodel,self.rdata,idx,pin.LOCAL_WORLD_ALIGNED).angular
        accwbasis -= drift

        J = np.vstack(Jfeet+[Jbasis,Jwbasis])
        acc = np.concatenate(afeet+[accbasis,accwbasis])
        
        x_err = np.concatenate(pfeet_err+[e_basispos,e_basisrot])
        dx_ref = np.concatenate(vfeet_ref+[self.base_linearvelocity_ref,self.base_angularvelocity_ref])

        invJ = self.dinv(J) #or np.linalg.inv(J) since full rank

        ddq = invJ @ acc
        self.q_cmd = pin.integrate(self.robot.model,q,invJ @ x_err)
        self.dq_cmd = invJ @ dx_ref
        
        return ddq




print("test")
dt = 0.001
invKin = Solo12InvKin() 
q = invKin.robot.q0.copy()
dq = invKin.robot.v0.copy()

if USE_VIEWER:
    invKin.robot.initViewer(loadModel=True)
    invKin.robot.viewer.gui.setRefreshIsSynchronous(False)


    #invKin.robot.display(q)


for i in range(1000):
    t=i*dt
    #set the references
    invKin.feet_position_ref = [
            np.array([0.1946,   0.16891, 0.0191028]),
            np.array([0.1946,  -0.16891, 0.0191028]),
            np.array([-0.1946,   0.16891, (1-np.cos(10*t))*.1+0.0191028]),
            np.array([-0.1946,  -0.16891, 0.0191028])]

    invKin.feet_velocity_ref = [
            np.array([0,0,0.]),
            np.array([0,0,0.]),
            np.array([0,0, np.sin(10*t)]),
            np.array([0,0,0.])]

    invKin.feet_acceleration_ref = [
            np.array([0,0,0.]),
            np.array([0,0,0.]),
            np.array([0,0, 10*np.cos(10*t)]),
            np.array([0,0,0.])]

    ddq = invKin.compute(q,dq)
    dq=dq+dt*ddq
    q=pin.integrate(invKin.robot.model,q,dq*dt)
    
    if USE_VIEWER:
        invKin.robot.display(q)
    #print (q)