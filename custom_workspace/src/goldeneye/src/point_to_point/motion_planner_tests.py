from motion_planner import plan_path

def test_planner():
    obstacles = [[5,0,1], [10,2,1], [15, -1, 1]]
    goal = [20,0]
    global X,Y,Psi
    X = 0
    Y = 0
    Psi = 0
    waypoints = plan_path(obstacles,goal, X, Y, Psi)
    print(waypoints)



if __name__ == '__main__':
    test_planner()
