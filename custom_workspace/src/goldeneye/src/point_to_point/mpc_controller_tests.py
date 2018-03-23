from mpc_controller import mpc_controller
import numpy as np

def test_mpc():
    Psi = -.65
    X = 3.13
    Y = 2.16
    targetX =4.47 
    targetY = 1.82
    V = 0.5
    delta = mpc_controller(Psi,X,Y,targetX,targetY,V)
    K = 400/(np.pi/6)
    print(delta*(180/np.pi) ,1500 - K*delta) 
 
if __name__ == '__main__':
    test_mpc()
