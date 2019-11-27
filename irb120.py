import pybullet as pb
import time
import pybullet_data

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

"""
def main():

        print(pb.getNumJoints(bot))
        pb.setJointMotorControlArray(bot, range(pb.getNumJoints(bot)), pb.POSITION_CONTROL,targetPositions=[1.0, 0.51,0.1,0.1,0.0,0.0,0,0])
        #print(pb.getBasePositionAndOrientation(bot))
        for _ in range(10000):
                pb.stepSimulation()
                time.sleep(0.1)
        print(pb.getBasePositionAndOrientation(bot))        
"""

def move(bot, torques):
       
        #pb.setJointMotorControl2(bot, 4, pb.VELOCITY_CONTROL, force= 0)
        pb.setJointMotorControlArray(bot, range(pb.getNumJoints(bot)), pb.VELOCITY_CONTROL, forces= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        pb.setJointMotorControlArray(bot, range(pb.getNumJoints(bot)), pb.TORQUE_CONTROL, forces= torques)
        for _ in range(2):
                pb.stepSimulation()
                jp , jv, _ = getJointStates(bot)
                print "states:", jp
                print "Joint Vel:",jv
                time.sleep(3)
        for i in range(pb.getNumJoints(bot)):
                pb.resetJointStateMultiDof(bot, i, targetValue= [0.0], targetVelocity= [0.0])
        jp , jv, _ = getJointStates(bot)
        print "states:", jp
        print "Joint Vel:",jv
        time.sleep(5)                
        
            
if __name__ == '__main__':
        torque = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        move(bot,torque)
        pb.disconnect()
        