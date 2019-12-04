import gym
from irb_openai import OpenaiIRB
import time

if __name__ == "__main__":

    env = OpenaiIRB()
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
