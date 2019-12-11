import tensorflow.compat.v1 as tf

tf.disable_v2_behavior()
import gym
import irb_pybullet
import time
import numpy as np


env = gym.make('irb_pybullet-v0')
action_max = env.action_space.high # update required, ongoing
action_min = env.action_space.low
num_states = env.observation_space.shape[0]
num_actions = env.action_space.shape[0]

with tf.Session() as sess:    
    saver = tf.train.import_meta_graph('QNet.meta')
    saver.restore(sess,tf.train.latest_checkpoint('./'))
    
    def get_action(s, noise_scale):
        a = sess.run(mu, feed_dict={X: s.reshape(1,-1)})[0]
        a += noise_scale * np.random.randn(num_actions)
        a =list(a)
        print("agent unscaled: ", a)
        k = 0
        for z in range(6):
            a[z] = (a[z]/2)*(action_max[z]-action_min[z])
            a[z] = np.maximum(np.minimum(a[z], action_max[z]), action_min[z])
        return  a  #np.clip(a, action_min, action_max)
    
    s = env.reset()
    a = get_action(s, 0.1)
    env.step(a)
    time.sleep(5)


    