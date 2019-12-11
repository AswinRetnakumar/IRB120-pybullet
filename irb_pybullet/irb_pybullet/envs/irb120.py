import pybullet as pb
import time
import pybullet_data
import numpy as np

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)
goal = [0.2665122782069116, 0.40988654527750035, 0.16178715865384]
#pb.setTimeStep(20)
pb.setRealTimeSimulation(0)


def getJointStates(robot):
  joint_states = pb.getJointStates(robot, range(pb.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def main():

        
        pb.addUserDebugLine(goal, goal, [1, 0, 0])
        print(pb.getNumJoints(bot))
        pb.setJointMotorControlArray(bot, range(6), pb.POSITION_CONTROL,targetPositions=[2.0868742, 0.99862355, 0.8, 1.0039154, 0.9852633, 1.0926043])
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
