from controller import simulate 
import numpy as np
import matplotlib.pyplot as plt  

def step_ref(goal, i, ref_v):
    if int(i/800)%2 == 0:
        goal[1] -= ref_v
    else:
        goal[1] += ref_v
    return goal

def get_actual_goal(x, ref_goal):
    if True :
        act_goal = [ref_goal[0]-6, ref_goal[1], ref_goal[2]]
    else:
        pass
    return act_goal

if __name__ == "__main__":
    x = [0,0,0,0,0,0,0,0,0,0,0,0]

    ref_goal = [6,0,0]

    t = 0
    dt = 0.05

    ref_v = 0.3

    res = []
    # diff = []
    for i in range(1000):
        res.append(np.concatenate(([i],x, ref_goal)))

        x = simulate(x, ref_goal, dt)
        ref_goal = step_ref(ref_goal,i, ref_v)       
    
    res = np.array(res)
    plt.figure(0)
    plt.plot(res[:,0], res[:,3], 'r')
    plt.plot(res[:,0], res[:,14], 'b')
    plt.figure(1)
    plt.plot(res[:,0], np.abs(res[:,3]-res[:,14]))
    plt.show()
       

