import gym
import irb_pybullet
import time

if __name__ == "__main__":

    env = gym.make("irb_pybullet-v0")
    print(env.action_space.sample())
    env.step(env.action_space.sample())
    time.sleep(1)
    env.step(env.action_space.sample())
    time.sleep(1)
    env.step(env.action_space.sample())
    time.sleep(1)
    env.step(env.action_space.sample())
    env.reset()
    time.sleep(1)
