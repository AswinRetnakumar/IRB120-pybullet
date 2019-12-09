import gym
import irb_pybullet
import time

if __name__ == "__main__":

    env = gym.make("irb_pybullet-v0")
    env.step([-1.7028723 , -0.52056706, -0.9624297 ,  0.88344127, -1.2120245 ,
       -0.6429722 ])
    for i in range(10000):
        print(env.action_space.sample())
        env.step(env.action_space.sample())
        print(i )

    env.reset()
