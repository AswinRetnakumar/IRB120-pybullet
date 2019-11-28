import pybullet as pb
import time
import pybullet_data
import numpy as np

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 


class irb():

    def __init__(self):
        
        pb.setGravity(0,0,-9.8)
        planeId = pb.loadURDF("plane.urdf")
        self.bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
        pb.setJointMotorControlArray(self.bot, range(pb.getNumJoints(self.bot)), pb.VELOCITY_CONTROL, forces= [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0])
        jp , jv, _ = self.getJointStates()
       
        self.init_pos = jp
        self.init_vel = jv
        self.goal = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.state = self.init_pos + self.init_vel
        self.vel = self.init_vel
        self.pos = self.init_pos
       
        self.error = np.sum(np.subtract(self.goal,self.state))
        self.done = False
        print "states:", self.state

    def reset(self):

        for i in range(pb.getNumJoints(self.bot)):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= [self.init_pos[i]], targetVelocity = [self.init_vel[i]])
        
    def set_state(self, pos, vel):

        for i in range(pb.getNumJoints(self.bot)):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= [pos[i]], targetVelocity = [vel[i]])


    def getJointStates(self):

        joint_states = pb.getJointStates(self.bot, range(pb.getNumJoints(self.bot)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def step(self,torques):
               
        pb.setJointMotorControlArray(self.bot, range(pb.getNumJoints(self.bot)), pb.TORQUE_CONTROL, forces= torques)
        pb.stepSimulation()
        time.sleep(0.01)
        jp, jv, _ = self.getJointStates()
        self.pos = jp
        self.state = jp + jv
        self.vel = jv
        self.error = np.sum(np.subtract(self.goal,self.state))
        if self.error == 0:
            self.done = True


        return self.state, self.vel, self.error, self.done
