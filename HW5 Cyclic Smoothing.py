import numpy as np
import math
import copy
path = [[0,0], #fix
        [1,0],
        [2,0],
        [3,0],
        [4,0],
        [5,0],
        [6,0], #fix
        [6,1],
        [6,2],
        [6,3], #fix
        [5,3],
        [4,3],
        [3,3],
        [2,3],
        [1,3],
        [0,3], #fix
        [0,2],
        [0,1]]
fix = [1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0]
# path = [[1, 0], # Move in the shape of a plus sign
#              [2, 0],
#              [2, 1],
#              [3, 1],
#              [3, 2],
#              [2, 2],
#              [2, 3],
#              [1, 3],
#              [1, 2],
#              [0, 2], 
#              [0, 1],
#              [1, 1]]
def smooth(path, fix, weight_param = 0.1, smoothing_param = 0.1, tol = 0.000001):
    smooth_path = copy.deepcopy(path)
    a = tol
    while a >= tol:
        a=0
        for i in range(len(path)):
            if fix[i] is not 1:
                for j in range(2):
                    aux = smooth_path[i][j]
                    smooth_path[i][j] += weight_param*(path[i][j]-smooth_path[i][j]) 
                    smooth_path[i][j] += smoothing_param*(smooth_path[(i-1)%len(path)][j]+smooth_path[(i+1)%len(path)][j]-2*smooth_path[i][j]) +\
                                        (0.5*smoothing_param)*(2*smooth_path[(i+1)%len(path)][j]-smooth_path[(i+2)%len(path)][j] - smooth_path[i][j]) +\
                                            (0.5*smoothing_param)*(2*smooth_path[(i-1)%len(path)][j] - smooth_path[(i-2)%len(path)][j] - smooth_path[i][j])
                    a += abs(aux-smooth_path[i][j])

    return smooth_path

new_path = smooth(path,fix)
for i in range(len(new_path)):
    print(new_path[i])


def close_enough(new,old,epsilon=0.01):
    if abs(new - old) > epsilon:
        return False
    return True

def solution_check(newpath,path):
    if type(newpath) != type(path):
        print("Type Error")
        return False
    if len(newpath) != len(path):
        print("Length Error")
        return False
    if len(newpath[0]) != len(path[0]):
        print("Length Error")
        return False
    for i in range(len(newpath)):
        for j in range(len(newpath[0])):
            
            if close_enough(new_path[i][j],path[i][j]) is False:
                print("Error")
                print(i,j)
                return False
    print("Test Correct!")
    return True
solution_check(new_path,path)