from motion_planner_v2 import plan_path

def test_planner():
    obstacles = [ [2,0,1],[3,6,1]]
    goal = [5,0]
    global X,Y,Psi
    X = 0
    Y = 0
    Psi = 0
    waypoints = plan_path(obstacles,goal, X, Y, Psi)
    print(waypoints)



if __name__ == '__main__':
    test_planner()
