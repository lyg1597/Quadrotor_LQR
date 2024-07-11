from controller import simulate 
import numpy as np
import matplotlib.pyplot as plt  
import matplotlib.animation as animation

prev_goal = None 
prev_r = 0

def prediction(prev_goal, prev_r, x):
    prev_r += 0.3 
    return prev_goal, abs(prev_r)

def compute_goal(c,r,x):
    cx, cy, cz = c 
    cx = cx-5-r 
    return [cx, cy, cz]

def step_ref(goal, i, ref_v):
    if int(i/600)%2 == 0:
        goal[1] -= ref_v
    else:
        goal[1] += ref_v
    return goal

def check_in_range(x, ref_goal):
    # if abs(ref_goal[0] - x[0]) > abs(ref_goal[1] - x[2]):
    #     return True 
    # else:
    #     return False
    if i>600 and i<700:
        return False
    else:
        return True

def get_actual_goal(x, ref_goal, i=0):
    global prev_goal 
    global prev_r
    if check_in_range(x, ref_goal):
    # if True:
        prev_goal = ref_goal
        prev_r = 0
        act_goal = [ref_goal[0]-5, ref_goal[1], ref_goal[2]]
    else:
        c,r = prediction(prev_goal, prev_r, x)
        prev_goal = c
        prev_r = r
        act_goal = compute_goal(c,r,x)
        print(act_goal)
        # act_goal = [ref_goal[0]-abs(ref_goal[1]-x[2]), ref_goal[1], ref_goal[2]]
    return act_goal

if __name__ == "__main__":
    x = [0,0,0,0,0,0,0,0,0,0,0,0]

    ref_goal = [5,0,0]

    t = 0
    dt = 0.05

    ref_v = 0.3

    res = []
    # diff = []
    for i in range(1000):
        if i==660:
            print("here")
        actual_goal = get_actual_goal(x, ref_goal, i)
        res.append(np.concatenate(([i],x, ref_goal, actual_goal)))
        x = simulate(x, actual_goal, dt)
        ref_goal = step_ref(ref_goal,i, ref_v)       
    
    res = np.array(res)
    plt.figure(0)
    plt.plot(res[:,0], res[:,3], 'r')
    plt.plot(res[:,0], res[:,17], 'b')
    plt.title('y')
    plt.figure(1)
    plt.plot(res[:,0], np.abs(res[:,3]-res[:,14]))
    plt.title('dy')
    # plt.show()
    plt.figure(2)
    plt.plot(res[:,0], res[:,1], 'r')
    plt.plot(res[:,0], res[:,16], 'b')
    plt.figure(3)
    plt.plot(res[:,0], res[:,5], 'r')
    plt.plot(res[:,0], res[:,18], 'b')
    plt.title('z')

    # # plt.figure(2)
    # # SamplQe data: two sequences of positions for two points
    # positions1 = res[:,(1,3)]
    # positions2 = res[:,(13,14)]
    # positions3 = res[:,(16,17)]

    # # Convert to numpy arrays for easier handling
    # positions1 = np.array(positions1)
    # positions2 = np.array(positions2)
    # positions3 = np.array(positions3)

    # fig, ax = plt.subplots()
    # ax.set_xlim(-5, 20)
    # ax.set_ylim(10, -70)

    # point1, = ax.plot([], [], 'bo')  # Blue point
    # point2, = ax.plot([], [], 'go')  # Green point
    # point3, = ax.plot([], [], 'ro')  # Green point
    # cone, = ax.plot([], [], 'r-')    # Cone facing positive x direction

    # def init():
    #     point1.set_data([], [])
    #     point2.set_data([], [])
    #     point3.set_data([], [])
    #     cone.set_data([], [])
    #     return point1, point2, point3, cone

    # def update(frame):
    #     # Update points
    #     point1.set_data(positions1[frame, 0], positions1[frame, 1])
    #     point2.set_data(positions2[frame, 0], positions2[frame, 1])
    #     point3.set_data(positions3[frame, 0], positions3[frame, 1])

    #     # Update cone
    #     x, y = positions1[frame]
    #     cone_length = 10
    #     cone_width = 10
    #     cone_x = [x, x + cone_length, x + cone_length, x]
    #     cone_y = [y, y - cone_width, y + cone_width, y]
    #     cone.set_data(cone_x, cone_y)
        
    #     return point1, point2, cone

    # ani = animation.FuncAnimation(fig, update, frames=len(positions1), init_func=init, blit=True)

    # ani.save('animation.mp4', writer='ffmpeg')

    plt.show()
        

