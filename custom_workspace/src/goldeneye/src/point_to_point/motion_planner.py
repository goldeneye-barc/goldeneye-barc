#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from math import sin, cos
import rrt_star 
import numpy as np
from marvelmind_nav.msg import hedge_pos

flag = rospy.get_param('heading_angle_flag')
while flag:
	flag = rospy.get_param('heading_angle_flag')

def get_xy(msg):
    global X,Y
    X = msg.x_m
    Y = msg.y_m
    
def get_Psi(msg):
    global Psi
    Psi = msg.z

def plan_path(obstacles, goal, X, Y, Psi):
    obstacles.sort(key=lambda obs: (obs[0] - X)**2 + (obs[1] - Y)**2)
    x_curr, y_curr, psi_curr = X, Y, Psi
    R_max = 1.0
    waypoints = []
    for obstacle in obstacles:
        dist = ((obstacle[0] - x_curr)**2 + (obstacle[1] - y_curr)**2) ** 0.5
        goal_dist = ((goal[0] - x_curr)**2 + (goal[1] - y_curr)**2) ** 0.5

        if dist < 2 * R_max:
            #print('less')
            theta = np.arccos(dist / (2 * R_max))
            theta = pnp.pi/2 - theta
            thetas = np.linspace(curr_psi-theta, curr_psi + theta, 21)
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
    next_pos = Vector3()
    next_pos.x = goal[0]
    next_pos.y = goal[1]
    waypoints.append(next_pos)
    return waypoints

plan_path = rrt_star.plan_path

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
