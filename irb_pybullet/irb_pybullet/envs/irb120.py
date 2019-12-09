import pybullet as pb
import time
import pybullet_data
import numpy as np

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
#pb.setTimeStep(20)
pb.setRealTimeSimulation(0)


def getJointStates(robot):
  joint_states = pb.getJointStates(robot, range(pb.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def main():

        print(pb.getNumJoints(bot))
        pb.setJointMotorControlArray(bot, range(6), pb.POSITION_CONTROL,targetPositions=[ 1.4295368 , -0.6637281 ,  0.63082004,  0.47785744,  0.26692984,
       -0.44841298])
        #print(pb.getBasePositionAndOrientation(bot))
        #pb.stepSimulation()
        for _ in range(100):
                pb.stepSimulation()
                time.sleep(0.1)
        time.sleep(2)
        pos,ore,_,_, _, _ = pb.getLinkState(bot, 6)
        pose = list(pos)+list(ore)
        print "Init pose: ", pose       
           
if __name__ == '__main__':
        pos,ore,_,_, _, _ = pb.getLinkState(bot, 6)
        print "init pos and orientation", pos, " ", ore
        #print "joint limits: ", pb.getJointInfo(bot,6)[7:10]
        
        main()
