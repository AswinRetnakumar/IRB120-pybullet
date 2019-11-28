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
    acc = [0.0]*8
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
    return A+ 1e-7
    

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
    return B+ 1e-7

    

def calc_lqr_input(env, sim_env):

    # get the current state from the true env
    state=copy.deepcopy(env.state)
    goal=copy.deepcopy(env.goal)
    Q = np.zeros((16, 16))
    Q[:8, :8] = np.eye(8) *1000
    R = np.eye(8) * 0.001

    sim_u=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  
    A=approximate_A(sim_env, state, sim_u, delta=1e-5)
    B=approximate_B(sim_env, state, sim_u, delta=1e-5) 
    #print "A: ", A
    #print "B: ", B
    P = la.solve_continuous_are(A, B, Q, R)
    BT_P=np.matmul(np.matrix.transpose(B),P)
    K=np.matmul(np.linalg.inv(R),BT_P)
    u=-np.matmul(K,np.subtract(state,goal)) 
    #u1 = []
    '''
    i = 0
    for n in u:
        print n
        u[i] = clamp(n,-1, 1)
        i += 1
    '''
    print 'u: ', u 

    return u, A, B