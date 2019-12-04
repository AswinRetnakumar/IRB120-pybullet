import pybullet as pb
import time
import pybullet_data
import numpy as np

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)

def getJointStates(robot):
  joint_states = pb.getJointStates(robot, range(pb.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def main():

        print(pb.getNumJoints(bot))
        pb.setJointMotorControlArray(bot, range(6), pb.POSITION_CONTROL,targetPositions=[0.0, 0.0,-2.7,0,0.0,1])
        #print(pb.getBasePositionAndOrientation(bot))
        
        for _ in range(100):
                pb.stepSimulation()
                time.sleep(0.1)
        pos,ore,_,_, _, _ = pb.getLinkState(bot, 6)
        pose = list(pos)+list(ore)
        print "Init pose: ", pose       
           
if __name__ == '__main__':
        pos,ore,_,_, _, _ = pb.getLinkState(bot, 6)
        print "init pos and orientation", pos, " ", ore
        print "joint limits: ", pb.getJointInfo(bot,3)[7:10]
        
        main()
