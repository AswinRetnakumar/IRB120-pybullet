import pybullet as pb 

joint_positions = [j[0] for j in pb.getJointStates(bot, range(6))]
print(joint_positions)