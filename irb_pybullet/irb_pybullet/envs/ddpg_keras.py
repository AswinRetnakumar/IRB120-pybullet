import numpy as np
import time
import gym
import matplotlib.pyplot as plt
from datetime import datetime
import irb_pybullet
from keras.models import Sequential,Model
from keras.layers import Flatten , Dense, Activation, Input
import tf.compat.v1 as tf
from keras import backend as K

sess = tf.Session()

K.set_session(sess)

def Network(s, layer_size, hidden_activation='relu', output_activation = 'tanh'):
    
    x = Dense(12, activation = hidden_activation)(s)
    for i in layer_size[1:-1]:
        x = Dense(i, activation = hidden_activation)(x)
    x = Dense(layer_size[-1], activation = output_activation)(x)
    
    return x

def Create_Net(s, action, layer_size, num_layers, action_size):
     
    with tf.variable_scope('mu'):
         mu = Network(s, layer_size)
    with tf.variable_scope('q'):
        input_ = tf.concat([s, action], axis=-1) # (state, action)
        q = tf.squeeze(Network(input_, layer_size+[1]), axis=1)
    with tf.variable_scope('q', reuse=True):
        # reuse is True, so it reuses the weights from the previously defined Q network
        input_ = tf.concat([s, mu], axis=-1) # (state, network actions)
        q_mu = tf.squeeze(Network(input_, layer_size+[1]), axis=1)
    
    return mu, q, q_mu









         

