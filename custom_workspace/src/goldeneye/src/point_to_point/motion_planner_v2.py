#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from math import sin, cos
import numpy as np
from marvelmind_nav.msg import hedge_pos

def get_xy(msg):
    global X,Y
    X = msg.x_m
    Y = msg.y_m
    
def get_Psi(msg):
    global Psi
    Psi = msg.z


def plan_path(obstacles, goal, X, Y, Psi):
    x_curr, y_curr, psi_curr = X, Y, Psi
    R_max = 1.0

    initial_pos = Vector3()
    initial_pos.x = x_curr
    initial_pos.y = y_curr
    waypoints = []
    waypoints.append(initial_pos)
   
    while (waypoints[-1].x != goal[0]) or (waypoints[-1].y != goal[1]):
        x_curr = waypoints[-1].x
        y_curr = waypoints[-1].y
        #sort obstacles based on distance from me
        obstacles.sort(key=lambda obs: (obs[0]-x_curr)**2 + (obs[1] - y_curr)**2)
        obstruct = False
        goal_vec = np.array([goal[0]-x_curr,goal[1]-y_curr])
        goal_vec = goal_vec /np.linalg.norm(goal_vec)
        goal_angle = np.arctan(goal_vec[1]/goal_vec[0])
        # check find closest obstacle that obstructs goal\
        for obs0 in obstacles:
            purp = np.cross(goal_vec,np.array([0,0,1]))

            head_down = obs0 - obs0[2]*purp
            head_up = obs0 + obs0[2]*purp
        
            up_ang = np.arctan((head_up[0]-x_curr)/(head_up[1]-y_curr))
            down_ang = np.arctan((head_down[0]-x_curr)/(head_down[1]-y_curr))
            
            if up_ang <= goal_angle and down_ang >= goal_angle and obstruct == False:
                obstruct = True
                obstacle = obs0

        if obstruct == True: 

            dist = ((obstacle[0] - x_curr)**2 + (obstacle[1] - y_curr)**2) ** 0.5
            goal_dist = ((goal[0] - x_curr)**2 + (goal[1] - y_curr)**2) ** 0.5

            if dist < 2 * R_max:
                #print('less')
                theta = np.arccos(dist / (2 * R_max))
                theta  = np.pi/2 - theta
                thetas = np.linspace(psi_curr -theta,psi_curr + theta, 21)
            else:
                #print('more')
                thetas = np.linspace(0, 2*np.pi, 50)

            min_dist = float('inf')#goal_dist
            best_psi = -float('inf')
            x_opt, y_opt = 0, 0
            for theta in thetas:
                xt = dist * np.cos(theta) + x_curr
                yt = dist * np.sin(theta) + y_curr

                curr_goal_dist = ((xt - goal[0])**2 + (yt - goal[1])**2) ** 0.5
                final_obstacle_dist = ((xt - obstacle[0])**2 + (yt - obstacle[1])**2) ** 0.5
                #print(curr_goal_dist, goal_dist, final_obstacle_dist, obstacle, theta)
                #print(curr_goal_dist < min_dist, final_obstacle_dist > obstacle[2])
                if curr_goal_dist < min_dist and final_obstacle_dist > obstacle[2]:
                    x_opt, y_opt, min_dist, best_psi = xt, yt, curr_goal_dist, theta
            #print(dist, min_dist, obstacle, 'fin')
            next_pos = Vector3()
            next_pos.x = x_opt 
            next_pos.y = y_opt
            x_curr, y_curr, psi_curr = x_opt, y_opt, best_psi
            waypoints.append(next_pos)

        else: 
            goalv = Vector3()
            goalv.x = goal[0]
            goalv.y = goal[1]
            waypoints.append(goalv)

    del waypoints[0]
    return waypoints


def preprocess(params):
    #  launch file string arguments as goal:="x,y" obstacles:="x1,y1,r1;x2,y2,r2"
    #goall
    goal_str = params['goal'].replace(" ", "")
    goal_x, goal_y = map(float, goal_str.split(','))
    params['goal'] = (goal_x, goal_y)

    #obstacles
    def process_obstacle(obs):
        return map(float, obs.split(','))

    obstacles_str = params['obstacles'].replace(" ", "")
    if params['obstacles'] == 'x': 
        obstacles = []
    else:
        obstacles = [process_obstacle(obstacle) for obstacle in obstacles_str.split(';')]
    params['obstacles'] = obstacles
    return params

def motion_planner():
    global X,Y,Psi
    X, Y, Psi, waypoints = None, None, None, None
    param_names = ['goal', 'obstacles'] 
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    params = preprocess(params)
    print(params)

    rospy.init_node('motion_planner')
    rospy.Subscriber('hedge_pos', hedge_pos,get_xy,queue_size=10)
    rospy.Subscriber('euler_angles',Vector3,get_Psi,queue_size=10)
    pub = rospy.Publisher('target_position',Vector3, queue_size=10)
    rate = rospy.Rate(10)

    curr_waypoint = 0
    thresh = 0.2

    while not rospy.is_shutdown():# and r.successful():
        if X is None or Y is None or Psi is None: continue
        if not waypoints:
            waypoints = plan_path(params['obstacles'], params['goal'], X, Y, Psi) # np.array of [(x1, y1, r1), (x2, y2, r3) ...] ; (x, y)
            print("Waypoints", waypoints)

        x_curr = waypoints[curr_waypoint].x
        y_curr = waypoints[curr_waypoint].y
        
        if ((X - x_curr)**2 + (Y - y_curr)**2) ** 0.5 < thresh:
            curr_waypoint = min(curr_waypoint + 1, len(waypoints)- 1)

        #print(waypoints[curr_waypoint])
        pub.publish(waypoints[curr_waypoint])
        rate.sleep()


if __name__ == '__main__':
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
