# 3D Control of Quadcopter
# based on https://github.com/juanmed/quadrotor_sim/blob/master/3D_Quadrotor/3D_control_with_body_drag.py
# The dynamics is from pp. 17, Eq. (2.22). https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
# The linearization is from Different Linearization Control Techniques for
# a Quadrotor System (many typos)

import dynamics
from dynamics import g, m, Ix, Iy, Iz
import numpy as np
import scipy
from scipy.integrate import odeint
import matplotlib.pyplot as plt 

def lqr(A, B, Q, R):
    """Solve the continuous time lqr controller.
    dx/dt = A x + B u
    cost = integral x.T*Q*x + u.T*R*u
    """
    # http://www.mwm.im/lqr-controllers-with-python/
    # ref Bertsekas, p.151

    # first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R) * (B.T * X))

    eigVals, eigVecs = scipy.linalg.eig(A - B * K)

    return np.asarray(K), np.asarray(X), np.asarray(eigVals)

# The control can be done in a decentralized style
# The linearized system is divided into four decoupled subsystems

# X-subsystem
# The state variables are x, dot_x, pitch, dot_pitch
Ax = np.array(
    [[0.0, 1.0, 0.0, 0.0],
     [0.0, 0.0, g, 0.0],
     [0.0, 0.0, 0.0, 1.0],
     [0.0, 0.0, 0.0, 0.0]])
Bx = np.array(
    [[0.0],
     [0.0],
     [0.0],
     [1 / Ix]])

# Y-subsystem
# The state variables are y, dot_y, roll, dot_roll
Ay = np.array(
    [[0.0, 1.0, 0.0, 0.0],
     [0.0, 0.0, -g, 0.0],
     [0.0, 0.0, 0.0, 1.0],
     [0.0, 0.0, 0.0, 0.0]])
By = np.array(
    [[0.0],
     [0.0],
     [0.0],
     [1 / Iy]])

# Z-subsystem
# The state variables are z, dot_z
Az = np.array(
    [[0.0, 1.0],
     [0.0, 0.0]])
Bz = np.array(
    [[0.0],
     [1 / m]])

# Yaw-subsystem
# The state variables are yaw, dot_yaw
Ayaw = np.array(
    [[0.0, 1.0],
     [0.0, 0.0]])
Byaw = np.array(
    [[0.0],
     [1 / Iz]])

####################### solve LQR #######################
Ks = []  # feedback gain matrices K for each subsystem
for A, B in ((Ax, Bx), (Ay, By), (Az, Bz), (Ayaw, Byaw)):
    n = A.shape[0]
    m = B.shape[1]
    Q = np.eye(n)
    Q[0, 0] = 10.  # The first state variable is the one we care about.
    R = np.diag([1., ])
    K, _, _ = lqr(A, B, Q, R)
    Ks.append(K)

####################### The controller ######################
def u(x, goal):
    # the controller
    UX = Ks[0].dot(np.array([goal[0], 0, 0, 0]) - x[[0, 1, 6, 7]])[0]
    UX = np.clip(UX, -0.5, 0.5)
    UY = Ks[1].dot(np.array([goal[1], 0, 0, 0]) - x[[2, 3, 8, 9]])[0]
    UY = np.clip(UY, -0.5, 0.5)
    UZ = Ks[2].dot(np.array([goal[2], 0]) - x[[4, 5]])[0]
    # UZ = np.clip(UZ, -0.4, 0.4)
    UYaw = Ks[3].dot(np.array([0, 0]) - x[[10, 11]])[0]
    UYaw = np.clip(UYaw, -0.5, 0.5)
    return np.array([UZ, UY, UX, UYaw])

######################## The closed_loop system #######################
def cl_nonlinear(x, t, goal):
    x = np.array(x)
    dot_x = dynamics.f_linear(x, u(x, goal) + np.array([m * g, 0, 0, 0]))
    return dot_x

def cl_linear(x, t, goal):
    x = np.array(x)
    dot_x = dynamics.f_linear(x, u(x, goal) + np.array([m * g, 0, 0, 0]))
    return dot_x

# simulate
def simulate(x, goal, dt):
    curr_position = np.array(x)[[0,2,4]]
    error = goal - curr_position
    distance = np.sqrt((error**2).sum())
    # if distance > 1:
    #     goal = curr_position + error / distance
    res = odeint(cl_nonlinear, x, [0, dt], args=(goal,))[-1]
    res[6] = res[6]%(np.pi*2)
    if res[6] > np.pi:
        res[6] -= 2*np.pi
    res[8] = res[8]%(np.pi*2)
    if res[8] > np.pi:
        res[8] -= 2*np.pi
    res[10] = res[10]%(np.pi*2)
    if res[10] > np.pi:
        res[10] -= 2*np.pi
    return res 

def step(goal,i, v):
    if int(i/800)%2 == 0:
        goal[1] -= v
    else:
        goal[1] += v
    return goal

if __name__ == "__main__":
    x = [0,0,0,0,0,0,0,0,0,0,0,0]

    ref_goal = [0,0,0]

    t = 0
    dt = 0.05

    ref_v = 0.3

    res = []
    # diff = []
    for i in range(1000):
        res.append(np.concatenate(([i],x, ref_goal)))
        x = simulate(x, ref_goal, dt)
        ref_goal = step(ref_goal,i, ref_v)       
    
    res = np.array(res)
    plt.figure(0)
    plt.plot(res[:,0], res[:,3], 'r')
    plt.plot(res[:,0], res[:,14], 'b')
    plt.figure(1)
    plt.plot(res[:,0], np.abs(res[:,3]-res[:,14]))
    plt.show()
       

