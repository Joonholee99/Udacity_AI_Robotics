import numpy as np

delta = [[-1, 0],
        [0,-1],
        [1,0],
        [0,1]]
delta_name = ['^','<','V','>']

def stochastic_value(grid, goal, cost_step, collision_cost, success_prob):
    failure_prob = (1 - success_prob)/2
    value = [[collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [['' for col in range(len(grid[0]))] for row in range(len(grid))]

    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                
                if x == goal[0] and y == goal[1]:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True
                        print("find Goal")

                elif grid[x][y] == 0:
                    for k in range(len(delta)):
                        stoch_move = [[-1,1,0],
                                    [failure_prob, failure_prob, success_prob]]
                        value2 = 0

                        if x + delta[k][0] >= 0 and y + delta[k][1] >= 0 and x+delta[k][0] < len(grid) and y+delta[k][1] < len(grid[0]) and value[x+delta[k][0]][y+delta[k][1]] != collision_cost:
                            for i in range(3):
                                    n = (k + stoch_move[0][i])%4

                                    x2 = x + delta[n][0]
                                    y2 = y + delta[n][1]

                                    if x2 < 0 or y2 < 0 or x2 > len(grid)-1 or y2 > len(grid[0])-1 :
                                        value_ = collision_cost
                                    else:
                                        value_ = value[x2][y2] 

                                    value2 += stoch_move[1][i] * value_

                            value2 += cost_step

                            
                            if value2 < value[x][y]:
                                value[x][y] = value2
                                
                                policy[x][y] = delta_name[k]
                                change = True 

                                # for row in range(len(value)):
                                #     print(policy[row])
                                # print('------------------------------------')
    
    return value, policy
                        


grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0], 
        [0, 1, 1, 0]]

goal = [0,len(grid[0])-1]
cost_step = 1
collision_cost = 1000
success_prob = 0.5

value,policy = stochastic_value(grid, goal, cost_step,collision_cost,success_prob)

for row in range(len(value)):
    print(value[row])
for row in range(len(policy)):
    print(policy[row])