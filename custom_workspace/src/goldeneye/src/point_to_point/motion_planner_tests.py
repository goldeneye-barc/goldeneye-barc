from motion_planner.py import plan_path

def test_planner():
    obstacles = ([5,0,1])
    goal = [10,0]
    global X,Y,Psi
    X = 0
    Y = 0
    Psi = 0
    waypoints = plan_path(obstacles,goal)
    print(waypoints)



if __name__ == '__main__':
    test_planner()
