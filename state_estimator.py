import numpy as np
import scipy as sp
import copy
from scipy import linalg as la

"""
TODO:

State vector should contain both the [position, velocity].
Hence the differential of state vector should also contain [velocity, accelrations] 

"""
time_step = 1/ 240
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

def simulate_dynamics(env, x, u):

    #record previous state
    #q=copy.deepcopy(env.pos)
    acc = [0.0]*6
    dstate = []
    dq=copy.deepcopy(env.vel)
    state=copy.deepcopy((env.state))
    env.state=copy.deepcopy(x)
    #env.set_state(env.pos, env.vel)
    _, dq1, _, _ = env.step(u) # dq1 is the present velocity use it for derivative of state
    #print 'dq1:', dq1
    acc = np.subtract(dq1,dq)
    #print "accelerations: ",acc
    acc *= 240
    env.state=copy.deepcopy(state)
    dstate = list(dq1) + list(acc)
    #print "dstate: ", dstate
    return dstate


def approximate_A(env, x, u, delta=1e-5):
        
    A=np.zeros((len(x),len(x)))
    #print "before change: ", A
    for i in range(len(x)):
        xp=copy.deepcopy(x)
        xn=copy.deepcopy(x)
        xp[i]=xp[i]+delta
        xn[i]=xn[i]-delta
        xdotp=simulate_dynamics(env,xp,u)
        xdotn=simulate_dynamics(env,xn,u)
        d_xdot = np.subtract(xdotp,xdotn)
        #print "length of d_xdot: ", d_xdot.shape
        dx= 2*delta
        A[:,i]= d_xdot/dx
    return A
    

def approximate_B(env, x, u, delta=1e-5):

    B=np.zeros((len(x),len(u)))
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
    

    sim_u=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  
    
    A=approximate_A(sim_env, state, sim_u, delta=1e-5)
    B=approximate_B(sim_env, state, sim_u, delta=1e-5) 
    print "A: ", A
    #print "B: ", B
    
    '''
    A = [[0,0.088385,-1.2609,-0.03054,0.452842,0,0,0,0,0,0,0],[0,40.15217,-29.5418, 0.108109, 1.133512,0,0,0,0,0,0,0],
        [0,-44.287,108.2842,-0.56953,-7.27049,0,0,0,0,0,0,0],[0,-0.12654,2.01236,1.614214,2.475777,0,0,0,0,0,0,0],
        [0,11.61132,-186.517,7.514058,129.9011,0,0,0,0,0,0,0],[0,0.066928,-1.24236,-1.58413,-2.83914,0, 0, 0, 0, 0, 0, 0],
        [0,0,0,0,0,0,1,0,0,0,0,0],[0,0,0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,0,0,0,1,0,0,0],[0,0,0,0,0,0,0,0,0,1,0,0],[0,0,0,0,0,0,0,0,0,0,1,0],[0,0,0,0,0,0,0,0,0,0,0,1]]

    B = [[16.52053, 0.075275, -0.35229, -12.2955, 3.515335, -1.98881],[0.075275, 3.888116, -8.51171, -0.11933, 11.05329, 0.073043],
        [-0.35229, -8.51171, 25.92747, 0.535638, -58.6871, -0.30182],[-12.2955, -0.11933, 0.535638, 315.0428, -3.43332, -304.069],
        [3.515335, 11.05329, -58.6871, -3.43332, 785.6144, 0.587171],[-1.98881, 0.073043, -0.30182, -304.069, 0.587171, 337005.8],
        [0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
    A = list(A)  
    B = list(B)
    
    A = np.subtract(A,np.zeros((12,12)))
    B = np.subtract(B, np.zeros((12, 6)))
    print "A: ", A 
    '''
    
    P = la.solve_continuous_are(A, B, env.Q, env.R)
    BT_P=np.matmul(np.matrix.transpose(B),P)
    K=np.matmul(np.linalg.inv(env.R),BT_P)
    error = np.subtract(state,goal)
    print "Error: ",error
    u = -np.matmul(K, error) 
    #u1 = []
    for i in range(4):
        u[i] = clamp(u[i], -1000, 1000)
    
    u[4] = clamp(u[4], -100, 100)
    u[5] = clamp(u[5],-10, 10)
    
    print "u: ",u
    return u, A, B