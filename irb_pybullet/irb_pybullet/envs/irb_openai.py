#!/usr/bin/env python
import numpy as np
import gym
import gym.spaces
from gym.envs.registration import register
import pybullet as pb
import time
import pybullet_data
import functools
import random

#physicsClient = pb.connect(pb.GUI)
physicsClient = pb.connect(pb.DIRECT)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
#pb.setTimeStep(0.0001)
#bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
pb.setRealTimeSimulation(0)

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
        #self._goal_pose = [0.2665122782069116, 0.40988654527750035, 0.16178715865384272 , 0.1685172244060629, 0.568363626537449, 0.16851711653306128, 0.7875066441262947]
        
        self.observation_space = gym.spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]),
            high=np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]))
        
        self.action_space = gym.spaces.Box(
            low  = np.array([-1.91, -1.4, -2.5, -2.0, -1.5, -1]), 
            high = np.array([1.91, 0.9, 0.8, 2.0, 1.5, 0.0]))
        
        self.bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
        self.distance_threshold = 0.001
        self.reward_type = 'dense'
        
        self.action_low  = [-1.91, -1.4, -2.5, -2.0, -1.5, -1] 
        self.action_high = [1.91, 0.9, 0.8, 2.0, 1.5, 0.0]
        #self.goal_pose = self.goal_gen()
        self.viewer = None
        self.state = None
        self.reset()

    def reset(self):
        
        self.goal_pose = self.goal_gen()
        self.q = list(self.init_q)
        self.state = list(self.init_pose)
        self.t = 0.0
        for i in range(6):
            pb.resetJointStateMultiDof(self.bot, i, targetValue= [self.init_q[i]], targetVelocity = [self.init_vel[i]]) # Manipulator goes to init state

        return np.array(self.state)
    
    '''
    def distance(self,state):

        p = []
        for i in range(len(self.goal_pose)):
            p.append(self.goal_pose[i] - state[i])

        return p

    '''


    def goal_distance(self, goal_a, goal_b):
        #assert goal_a.shape == goal_b.shape
        
        #Scaling for positional accuracy
        p = []
        for i in range(len(self.goal_pose)):
            if i<3:
                p.append((goal_a[i] - goal_b[i])*10)
            else:
                p.append(goal_a[i] - goal_b[i])
        print("p: ", p)
        return np.linalg.norm(p, axis=-1)
        '''
        d = np.linalg.norm(goal_a - goal_b, axis=-1)

        return d 
        '''

    def reward_compute(self, state):

        d = self.goal_distance(self.state, self.goal_pose)

        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            if d> self.distance_threshold:
                done = False
                return -d, done
            else:
                done = True
                return -d, done

            

    '''
    def reward_compute(self,state):
        
        threshold = 2
        threshold1 = 0.001
        done = False
        if np.sum( np.absolute(self.distance(state)) ) < threshold:
            reward = 10
        elif np.sum( np.absolute(self.distance(state)) ) < threshold1:
            reward = 100

        elif np.sum( np.absolute(self.distance(state)) ) == 0:
            reward = 1000
            done = True
        else:
            reward = -10
        
        return reward, done
    '''

    def goal_gen(self):
        
        action = [0.0]*6
        for i in range(6):
            action[i] = random.uniform(self.action_low[i], self.action_high[i])
        goal, _, _, _ = self.step(action, k= 1)
        
        return goal

    def step(self, action, k= 0):

        # Takes an action from algorithm, publish the data to ROS and collects pose data.
        # Compares pose with goal pose and computes reward and determines whether the current pose is same as goal pose
        # If yes done is set to true
        #print "step function"
        print("Action: ", list(action))
        pb.setJointMotorControlArray(self.bot, range(6), pb.POSITION_CONTROL,targetPositions= list(action))
        
        
        #pb.stepSimulation()
        
        curr = [0.0]*6
        
        while True:
            pb.stepSimulation()
            curr = [0.0]*6
            #time.sleep(0.01)
            for i in range(6):
                curr[i]= pb.getJointState(self.bot, i)[0]
            #print("Current: ", curr)
            if functools.reduce(lambda i, j : i and j, map(lambda m, k: (m-k)< 0.0001, action, curr), True):
                break
        
        pos,ore,_,_, _, _ = pb.getLinkState(self.bot, 6)
        self.state = np.array(list(pos)+list(ore))
        print("state: ", self.state)
        
        if k== 0:
            print("goal: ", self.goal_pose)
            reward,done = self.reward_compute(self.state)
            print('Reward: ',reward)
            print('\n\n')
        else:
            reward, done = None, None

        return self.state, reward, done, {}

          
    def render(self, mode='human', close=False):
        print("Dont know what to do....... ")

