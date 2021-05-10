import numpy as np 

forward = [[-1,0],
        [0,-1],
        [1,0],
        [0,1]]

forward_name = [['up'],
                ['left'],
                ['down'],
                ['right']]

action = [-1,0,1]
action_name = ['R','#','L']

grid = [[1,1,1,0,0,0],
        [1,1,1,0,1,0],
        [0,0,0,0,0,0],
        [1,1,1,0,1,1],
        [1,1,1,0,1,1]]

goal = [2,0]

cost = [2,1,20]
init = [4,3,0]
def optimum_policy2D(grid, init, cost):
    value = [[[999 for row in range(len(grid[0]))] for col in range(len(grid))],
            [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
            [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
            [[999 for row in range(len(grid[0]))] for col in range(len(grid))]]
    
    policy = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
            [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
            [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
            [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]
    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):
                    if x == goal[0] and y == goal[1]:
                        if value[orientation][x][y] > 0:
                            value[orientation][x][y] = 0
                            change = True
                            policy[orientation][x][y] = '*'

                    elif grid[x][y] == 0:
                        for i in range(3):
                            o2 = (orientation+action[i])%4
                            x2 = x + forward[o2][0]
                            y2 = y + forward[o2][1]
                            
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2]==0:
                                # print(value[o2][x2])
                                v2 = value[o2][x2][y2] + cost[i]
                                if v2 < value[orientation][x][y]:
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[i]
                                    change = True

    x = init[0]
    y = init[1]
    orientation = init[2]

    policy2D[x][y] = policy[orientation][x][y]

    while policy[orientation][x][y] != '*':
        if policy[orientation][x][y] == '#':
            o2 = (orientation)
        elif policy[orientation][x][y] == 'R':
            o2 = (orientation-1) % 4
        elif policy[orientation][x][y] == 'L':
            o2 = (orientation+1) % 4
        x = x + forward[o2][0]
        y = y + forward[o2][1]

        orientation = o2
        # print(o2)
        # print(o2,x,y)
        # print(policy2D[x][y])
        policy2D[x][y] = policy[orientation][x][y]

    for orientation in range(4):
        print('------------------------------')
        for i in range(len(policy[orientation])):
            print(policy[orientation][i])


optimum_policy2D(grid, init, cost)


