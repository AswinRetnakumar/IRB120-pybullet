#!/usr/bin/env python
import numpy as np
import gym
import gym.spaces
from gym.envs.registration import register
import pybullet as pb
import time
import pybullet_data


physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
#bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)


class OpenaiIRB(gym.core.Env):
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 15
    }

    def __init__(self,
                 goal_pose=None,
                 init_pose=None,
                 init_q=None,
                 ):
        """Observation space consists of pose data"""
        self.init_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.init_pose = [0.374, -2.439454888092385e-19, 0.6299999999999999, 0.0, 0.0, 0.0, 1.0 ]
	self.init_vel = [0.0]*6
        self._goal_pose = [0.2665122782069116, 0.40988654527750035, 0.16178715865384272 , 0.1685172244060629, 0.568363626537449, 0.16851711653306128, 0.7875066441262947]
        self.observation_space = gym.spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]),
            high=np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]))
        
        self.action_space = gym.spaces.Box(
            low  = np.array([-1.91, -1.91, -2.5, -np.pi/2, -5*np.pi/8, -2*np.pi]), 
            high = np.array([1.91, 0.9, 0.8, np.pi/2, 5*np.pi/8, 2*np.pi]))

        self.bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)

 
        self.viewer = None
        self.state = None
        self.reset()

    def reset(self):
        
        self.goal_pose = list(self._goal_pose)
        self.q = list(self.init_q)
        self.state = list(self.init_pose)
        self.t = 0.0
        for i in range(6):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= [self.init_q[i]], targetVelocity = [self.init_vel[i]]) # Manipulator goes to init state

        return np.array(self.state)
    
    def distance(self,state):

        p = []
        for i in range(len(self.goal_pose)):
            p.append(self.goal_pose[i] - state[i])

        return p


    def reward_compute(self,state):
        
        threshold = 0.001
        done = False
        if np.sum( np.absolute(self.distance(state)) ) < threshold:
            reward = 100

        else:
            reward = -10
        
        return reward, done
        

    def step(self, action):

        # Takes an action from algorithm, publish the data to ROS and collects pose data.
        # Compares pose with goal pose and computes reward and determines whether the current pose is same as goal pose
        # If yes done is set to true
        print "step function"
        pb.setJointMotorControlArray(self.bot, range(6), pb.POSITION_CONTROL,targetPositions= action)
        pb.stepSimulation()
        pos,ore,_,_, _, _ = pb.getLinkState(self.bot, 6)
        state = list(pos)+list(ore)
        reward,done = self.reward_compute(state)

        return self.state, reward, done, {}

          
    def render(self, mode='human', close=False):
        print("Dont know what to do....... ")
