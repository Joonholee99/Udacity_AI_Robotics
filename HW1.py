import numpy as np 

# Udacity <Artificial Intelligence for Robotics>
# Localization - Programming assignment
# 
colors = [['r','g','g','r','r'],['r','r','g','r','r'],
            ['r','r','g','g','r'],['r','r','r','r','r']]
measurements = ['g','g','g','g','g']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
sensor_right = 0.7
p_move = 0.8

#-----------------------------------------------------------
def grid_init(a,b):

    grid = np.zeros((a,b))
    print("grid size is ",np.shape(grid))
    grid_prob = np.ones((a,b))*(1/(a*b))
    print("grid probability initialized by",grid_prob[1][1])
    
    return grid_prob

def sense(G,s,colors):
    New_G = np.zeros((4,5))
    
    for i in range(len(G)):
        for j in range(len(G[0])):
            hit = (s == colors[i][j])
            New_G[i][j] = (hit)*sensor_right*G[i][j] + (1-hit)*(1-sensor_right)*G[i][j]

    alpha = np.sum(New_G)

    for i in range(len(G)):
        for j in range(len(G[0])):
            New_G[i][j] = New_G[i][j] / alpha

    return New_G

def move(G,m):

    New_G = np.zeros((4,5))
    print(m)

    for i in range(len(G)):
        for j in range(len(G[0])):
            New_G[i][j] = G[(i-m[0]) % len(G)][(j-m[1]) % len(G[0])]*p_move + G[i][j]*(1-p_move)
    
    # New_G = New_G / (np.sum(New_G))

    return New_G

# -----------------------------------------------------------

grid = grid_init(len(colors),len(colors[0]))


for i in range(len(measurements)):
    grid = move(grid,motions[i])
    print("After move")
    print(grid)
    grid = sense(grid, measurements[i],colors)
    print("After sense")
    print(grid)
    print("--------------------------------------------\n")