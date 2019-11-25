import pybullet as pb
import time
import pybullet_data

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
pb.setGravity(0,0,-9.8)
planeId = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("irb120_3_58.urdf",[0, 0, 0], useFixedBase=1)

def main():

    pb.setJointMotorControlArray(bot, range(6), pb.TORQUE_CONTROL,forces=[1600, 0.0 , 0 , 0, 0, 0])
    for _ in range(20):
        pb.stepSimulation()
        time.sleep(0.5)




def joint_states():

    joint_positions = [j[0] for j in pb.getJointStates(bot, range(6))]
    print(joint_positions)

def move(torques):
    raise NotImplementedError

if __name__ == '__main__':
    main()
    joint_states()