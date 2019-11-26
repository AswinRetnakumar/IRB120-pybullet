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

def main():

        print(pb.getNumJoints(bot))
        pb.setJointMotorControlArray(bot, range(pb.getNumJoints(bot)), pb.POSITION_CONTROL,targetPositions=[0.0, 0.0,0.0,0.0,0.0,0.0,0,0])
        for _ in range(10000):
                pb.stepSimulation()
                time.sleep(0.1)
                


def move(bot, torques):
        prev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(0,5000,10):
                #forces for torque cntrl and target_position s for pos control                                                        
                #base =[-590, 590],shoulder, elbow,                            
                pb.setJointMotorControlArray(bot, range(pb.getNumJoints(bot)), pb.TORQUE_CONTROL, forces= torques) # fourth state is unused
                                                                                                # i dont know why
                jp, jv, jt = getJointStates(bot)
                '''if (jp[1])< -1.5:
                        print(jp)
                        print(i)
                        pb.disconnect()
                        break
                '''
                prev = jp
                print("pos:", jp)
                print("vel:", jv)
                time.sleep(0.2)
                pb.stepSimulation()
        time.sleep(5)
    
if __name__ == '__main__':
    move(bot,[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    