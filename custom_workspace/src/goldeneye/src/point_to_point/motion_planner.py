#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from math import sin, cos
from marvelmind_nav.msg import hedge_pos

def get_xy(msg):
    global X,Y
    X = msg.x_m
    Y = msg.y_m
    
def get_Psi(msg):
    global Psi
    Psi = msg.z

def plan_path(obstacles, goal):
    obstacles = obstacles.sort(key=lambda obs: (obs[0] - X)**2 + (obs[1] - Y)**2)
    x_curr, y_curr, psi_curr = X, Y, Psi
    R_max = 1
    waypoints = []
    for obstacle in obstacles:
        dist = ((obstacle[0] - x_curr)**2 + (obstacle[1] - y_curr)**2) ** 0.5
        goal_dist = ((goal[0] - x_curr)**2 + (goal[1] - y_curr)**2) ** 0.5

        if dist < 2 * R_max:
            theta = np.acos(dist / (2 * R_max))
            thetas = np.linspace(-theta, theta, 21)
        else:
            thetas = np.linspace(0, 2*np.pi, 50)

        min_dist = goal_dist
        best_psi = -float('inf')
        x_opt, y_opt = 0, 0
        for theta in thetas:
            xt = dist * np.cos(theta) + x_curr
            yt = dist * np.sin(theta) + y_curr

            curr_goal_dist = ((xt - curr_x)**2 + (yt - curr_y)**2) ** 0.5
            final_obstacle_dist = ((xt - obstacle[0])**2 + (yt - obstacle[1])**2) ** 0.5
            if curr_goal_dist < goal_dist and final_obstacle_dist > obstacle[2]:
                x_opt, y_opt, min_dist, best_psi = xt, yt, curr_goal_dist, theta
        next_pos = Vector3()
        next_pos.x = x_opt 
        next_pos.y = y_opt
        x_curr, y_curr, psi_curr = x_opt, y_opt, best_psi
    return waypoints



def motion_planner():
    global X,Y,Psi
    X, Y, Psi = None, None, None

    rospy.init_node('motion_planner')
    rospy.Subscriber('hedge_pos',hedge_pos,get_xy,queue_size=10)
    rospy.Subscriber('/euler_angles',Vector3,get_Psi,queue_size=10)
    pub = rospy.Publisher('target_position',Vector3, queue_size=10)
    rate = rospy.Rate(10)
    while X is None or Y is None or Psi is None: continue

    target_pos = plan_path(obstacles, goal) # np.array of [(x1, y1, r1), (x2, y2, r3) ...] ; (x, y)
    while not rospy.is_shutdown():# and r.successful():
        pub.publish(target_pos)
        rate.sleep()


if __name__ == '__main__':
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
