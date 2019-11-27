import pybullet as pb
import time
import pybullet_data

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 


class irb():

    def __init__(self):
        
        pb.setGravity(0,0,-9.8)
        planeId = pb.loadURDF("plane.urdf")
        self.bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
        pb.setJointMotorControlArray(self.bot, range(pb.getNumJoints(self.bot)), pb.VELOCITY_CONTROL, forces= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        jp , jv, _ = self.getJointStates(self.bot)
        print "states:", jp
        self.init_pos = jp
        self.init_vel = jv
        self.goal = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pos = self.init_pos
        self.vel = self.init_vel
        self.error = self.goal - self.pos

    def reset(self):

        for i in range(pb.getNumJoints(self.bot)):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= self.init_pos[i], targetVelocity = self.init_vel[i])

    def set_state(self, state, vel):

        for i in range(pb.getNumJoints(self.bot)):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= state[i], targetVelocity = vel[i])


    def getJointStates(self):

        joint_states = pb.getJointStates(self.bot, range(pb.getNumJoints(self.bot)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def step(self,torques):
               
        pb.setJointMotorControlArray(self.bot, range(pb.getNumJoints(self.bot)), pb.TORQUE_CONTROL, forces= torques)
        pb.stepSimulation()
        jp, jv, _ = self.getJointStates(self.bot)
        self.pos = jp
        self.vel = jv
        self.error = self.goal - self.pos

        return self.pos, self.error, self.done
