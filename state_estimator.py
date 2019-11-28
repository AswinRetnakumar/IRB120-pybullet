import numpy as np
import scipy as sp
import copy
from scipy import linalg as la

"""
TODO:

State vector should contain both the [position, velocity].
Hence the differential of state vector should also contain [velocity, accelrations] 

"""

def simulate_dynamics(env, x, u):

    #record previous state
    #q=copy.deepcopy(env.pos)
    #dq=copy.deepcopy(env.vel)
    state=copy.deepcopy((env.state))
    env.state=copy.deepcopy(x)
    _, vels, _, _ = env.step(u)
    env.state=copy.deepcopy(state)
    return vels


def approximate_A(env, x, u, delta=1e-5):
        
    A=np.zeros((2*len(x),2*len(x)))
    for i in range(2*len(x)):
        xp=copy.deepcopy(x)
        xn=copy.deepcopy(x)
        xp[i]=xp[i]+delta
        xn[i]=xn[i]-delta
        xdotp=simulate_dynamics(env,xp,u)
        xdotn=simulate_dynamics(env,xn,u)
        d_xdot = np.subtract(xdotp,xdotn)
        dx=2*delta
        A[:,i]=d_xdot/dx
    return A
    

def approximate_B(env, x, u, delta=1e-5):

    B=np.zeros((len(x),len(x)))
    for i in range(len(u)):
        up=copy.deepcopy(u)
        un=copy.deepcopy(u)
        up[i]=up[i]+delta
        un[i]=un[i]-delta
        xdotp=simulate_dynamics(env,x,up)
        xdotn=simulate_dynamics(env,x,un)
        d_xdot = np.subtract(xdotp,xdotn)
        du=2*delta
        B[:,i]=d_xdot/du
    return B

    

def calc_lqr_input(env, sim_env):

    # get the current state from the true env
    state=copy.deepcopy(env.state)
    goal=copy.deepcopy(env.goal)
    Q = np.zeros((16, 16))
    Q[:8, :8] = np.eye(8) * 1000
    R = np.eye(8) * 0.001

    sim_u=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  
    A=approximate_A(sim_env, state, sim_u, delta=1e-5)

    B=approximate_B(sim_env, state, sim_u, delta=1e-5) 

    
    P = la.solve_continuous_are(A, B, Q, R)
    BT_P=np.matmul(np.matrix.transpose(B),P)
    K=np.matmul(np.linalg.inv(R),BT_P)
    u=-np.matmul(K,state-goal)    

    return u, A, B