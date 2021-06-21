from math import *
import numpy as np

def doit(init, move1, move2):
    Omega = np.array([[1,0,0],[0,0,0],[0,0,0]])
    Xi = np.array([[init],[0],[0]])

    Omega += np.array([[1,-1,0],[-1,1,0],[0,0,0]])
    Xi += np.array([[-move1],[move1],[0]])

    Omega += np.array([[0,0,0],[0,1,-1],[0,-1,1]])
    Xi += np.array([[0],[-move2],[move2]])

    print(Omega)
    print(Xi)
    print(np.dot(np.linalg.inv(Omega),Xi))

doit(-3,5,3)