from motion_planner_v2 import plan_path

def test_planner():
    obstacles = [[2.53, 2.42, 0.5]]
    goal = [3.06,4.38]
    global X,Y,Psi
    X = 2.4
    Y = 0.4
    Psi = 3.14159/2
    waypoints = plan_path(obstacles,goal, X, Y, Psi)
    print(waypoints)



if __name__ == '__main__':
    test_planner()
