import numpy as np

grid = [[0,0,1,0,0,0],
        [0,0,1,0,0,0],
        [0,0,1,0,0,0],
        [0,0,0,0,1,0],
        [0,0,1,1,1,0],
        [0,0,0,0,1,0]]

goal = [len(grid)-1, len(grid[0])-1]
# print(goal)
cost = 1

delta = [[-1,0],
        [1,0],
        [0,-1],
        [0,1]]

delta_name = ['^','v','<','>']

def compute_value(grid, goal, cost):
    x = goal[0]
    y = goal[1]
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    value[x][y] = 0
    Done = False
    expansion = []
    expansion.append([x, y])

    new_expansion = []
    action = [['' for row in range(len(grid[0]))] for col in range(len(grid))]
    action[x][y] = '*'
    change = True
    while change == True:
        change = False
        # print(cost)
        for j in range(len(expansion)):
            x = expansion[j][0]
            y = expansion[j][1]
            
            # if x == 0 and y == 0:
            #     Done = True
            #     print("Finish")
            
            for i in range(len(delta)):
                x_next = x + delta[i][0]
                y_next = y + delta[i][1]
                if i == 0:
                    a = 1
                elif i == 1:
                    a = 0
                elif i == 2:
                    a = 3
                elif i == 3:
                    a = 2

                if  (x_next >= 0 and y_next >= 0 and
                    x_next <= len(grid)-1 and y_next <= len(grid[0])-1):

                    if (value[x_next][y_next] == 99 and grid[x_next][y_next] == 0):

                        value[x_next][y_next] = cost
                        new_expansion.append([x_next, y_next])
                        action[x_next][y_next] = delta_name[a]
                        change = True
                        
        # print(new_expansion)
        expansion = new_expansion
        new_expansion = []
        cost += 1
        if change == False:
            print("Finish")
        # print(new_expansion)

    return value, action

value, action = compute_value(grid, goal, cost)

for i in range(len(value)):
    print(value[i])
for i in range(len(value)):
    print(action[i])