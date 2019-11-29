from irb120_class import irb
from state_estimator import simulate_dynamics,approximate_A,approximate_B,calc_lqr_input
import numpy as np
import copy
import time
import os


fp1='pos_log.txt'
fp2='vel_log.txt'
fp3='torque.txt'
fp4='cost.txt'

f1 = open(fp1, 'w')
f2 = open(fp2, 'w')
f3 = open(fp3, 'w')
f4 = open(fp4, 'w')


env = irb()
prev_state = env.reset()
print("Goal:",env.goal)
sim_env=copy.deepcopy(env)
done=False
rewards=0
cnt=0
AA = []
BB = []
while done==False:
    print(cnt)
    q=env.state
    f1.write(str(q)+'\n')
    dq=env.vel
    f2.write(str(dq)+'\n')
    u, A, B = calc_lqr_input(env, sim_env)
    AA.append(A)
    BB.append(B)
    #print('u:',u)
    f3.write(str(u)+'\n')
    x,vel,reward,done=env.step(u)
    rewards += np.sum(reward)
    f4.write(str(rewards)+'\n')
    #print("rewards:",rewards)
    #print(done)
    cnt=cnt+1
f1.close()
f2.close()
f3.close()
f4.close()