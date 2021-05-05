grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0,0]
goal = [len(grid)-1 , len(grid[0])-1]
cost = 1

delta = [[-1,0],
        [0,-1],
        [1,0],
        [0,1]]
print(delta)
delta_name = ['^', '<', 'v', '>']

def optimum_policy(grid, goal, cost):
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [['' for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True
    count = 0

    while change:
        change = False
        count+=1
        # print(count)
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                
                if goal[0] == x and goal[1] == y:
                    if value[x][y] >0:
                        value[x][y] = 0
                        action[x][y] = '*'
                        change  = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if  x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2]==0:
                            v2 = value[x2][y2] + cost

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2 
                                action[x][y] = delta_name[a]
                                


    return value, action

value1, action1 = optimum_policy(grid, goal, cost)

for i in range(len(value1)):
    print(value1[i])
for i in range(len(action1)):
    print(action1[i])