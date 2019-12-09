import gym
import irb_pybullet
import time

if __name__ == "__main__":

    env = gym.make("irb_pybullet-v0")
    env.step([0.99828935, -0.8599756 ,  0.5259806 , -1.2399071 ,  1.182837  ,
       -0.6918211])
    time.sleep(10)
    for i in range(10000):
        #print(env.action_space.sample())
        env.step(env.action_space.sample())
        print(i )

    env.reset()
