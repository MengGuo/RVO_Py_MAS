import sys


from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic


#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
#circular obstacles, format [x,y,rad]
ws_model['circular_obstacles'] = [[0.8,1.5,0.4], [2.3,1.5,0.4], [3.8,1.5,0.4], [1.0,3.5,0.7], [4.0,3.5,0.7]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for robot 
# position of [x,y]
X = [[0.3,0.0], [0.9,0.0], [1.5,0.0], [2.1,0.0], [2.7,0.0], [3.3,0.0], [3.9,0.0],
     [0.3,5.0], [0.9,5.0], [1.5,5.0], [2.1,5.0], [2.7,5.0], [3.3,5.0], [3.9,5.0]]
# velocity of [vx,vy]
V = [[0,0] for i in xrange(len(X))]
# maximal velocity norm
V_max = [1.0 for i in xrange(len(X))]
# goal of [x,y]
goal = [[0.3,5.0], [0.9,5.0], [1.5,5.0], [2.1,5.0], [2.7,5.0], [3.3,5.0],[3.9,5.0],
        [0.3,0.0], [0.9,0.0], [1.5,0.0], [2.1,0.0], [2.7,0.0], [3.3,0.0], [3.9,0.0],]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 10
# simulation step
step = 0.1

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in xrange(len(X)):
        if reach(X[i], goal[i], 0.1):
            V[i][0] = 0
            V[i][1] = 0
        else:
            X[i][0] += V[i][0]*step
            X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    visualize_traj_dynamic(ws_model, X, V, goal, time=t, name='data/snap%s.png'%str(t))
    t += 1
    
