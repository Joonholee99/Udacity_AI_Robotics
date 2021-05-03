import numpy as np

grid = [[0,1,1,0,0,0],
        [0,1,1,0,0,0],
        [0,1,0,0,0,0],
        [0,1,0,0,0,0],
        [0,0,0,0,0,0]]

init = [0,0]

goal = [len(grid)-1, len(grid[0])-1]
cost = 1
heuristic = []
for i in range(len(grid)):
    r_heuristic = []
    for j in range(len(grid[0])):
        h = -i+(goal[0]) - j+(goal[1])
        r_heuristic.append(h)
    
    heuristic.append(r_heuristic)
    
delta = [[-1,0],
        [1,0],
        [0,-1],
        [0,1]]
delta_name = ['^','v','<','>']

def search(grid, init, goal, cost):

    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expansion = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    expansion[init[0]][init[1]] = 0
    count = 0

    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    path = [['' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic[x][y]
    open = [[f,g,x,y]]

    found= False # when we find
    resign = False # if we cannot find

    while found is False and resign is False:

        # 1. Check Open list available
        if len(open) == 0:
            resign = True
            print('Fail to Find')
        else:
            open.sort() #오름차순
            open.reverse() #내림차순
            next = open.pop() # Smallest G 뽑음

            x = next[2]
            y = next[3]
            g = next[1]
            f = next[0]
            expansion[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
                print(next)
                print("Searching Done with G Value is", g)

                for k in range(len(path)):
                    print(expansion[k])
                
            else:
                min_idx = []
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g+1
                            f2 = g2 + heuristic[x2][y2]
                            
                            open.append([f2,g2,x2,y2])
                            closed[x2][y2] = 1
                            
    return action


action = search(grid,init,goal,cost)

# path = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
# x = goal[0]
# y = goal[1]
# path[x][y] = '*'

# while x != init[0] or y != init[1]:
#     x2 = x - delta[action[x][y]][0]
#     y2 = y - delta[action[x][y]][1]
#     path[x2][y2] = delta_name[action[x][y]]
#     x = x2
#     y = y2    

# for i in range(len(path)):
#     print(path[i])

