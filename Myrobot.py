import numpy as np
import math
import random
import copy

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.steering_noise = 0
        self.steering_drift = 0
        self.length = 20
        self.distance_noise = 0
        self.measurement_noise = 0
        self.num_collision = 0
        self.num_steps = 0

    def set(self,x,y,orientation,length):
        self.x = x 
        self.y = y
        self.orientaiton = orientation
        self.length = length

    def set_noise(self, steering_noise, distance_noise, measurement_noise):
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

    def set_steering_drift(self, drift):
        self.steering_drift =drift

    def move(self, steering, distance, tol = 0.001, max_steering_angle=math.pi/4):
        if steering > max_steering_angle:
            steering = max_steering_angle
        elif steering < -max_steering_angle:
            steering = -max_steering_angle

        if distance < 0:
            distance = 0

        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        steering2 += self.steering_drift
        turn = math.tan(steering2) * distance2 / self.length

        res = copy.deepcopy(self)
        res.num_steps += 1
        if turn < tol:
            res.x += distance2 * math.cos(res.orientation)
            res.y += distance2 * math.sin(res.orientation)
            res.orientation += turn
        else:
            R = self.length / (math.tan(steering2))
            cx = res.x - R * math.cos(self.orientation)
            cy = res.y + R * math.sin(self.orientation)
            res.x = cx + R * math.cos(self.orientation + turn)
            res.y = cy - R * math.sin(self.orientation + turn)

        return res

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    len = math.sqrt((self.x - i)**2 + (self.y - j)**2)
                    if len < 0.5:
                        self.num_collision += 1
                        return False
        
        return True
    
    def check_goal(self, goal, threshold = 1):
        dist = math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)
        
        return dist < threshold

    def sense(self):
        return [random.gauss(self.x,self.measurement_noise), random.gauss(self.y, self.measurement_noise)]

    def measurment_prob(self, measurement):
        error_x = self.x - measurement[0]
        error_y = self.y - measurement[1]

        prob = (1/math.sqrt(2*math.pi*(self.measurement_noise**2))) * math.exp(-error_x/(2*(self.measurement_noise**2)))
        prob *= (1/math.sqrt(2*math.pi*(self.measurement_noise**2))) * math.exp(-error_y/(2*(self.measurement_noise**2)))

        return prob


def Astar_search(grid, init, goal):

    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expansion = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[1,0],
            [-1,0],
            [0,1],
            [0,-1]]
    action_p = ['V','^','>','<']
    heuristic = []

    for i in range(len(grid)):
        r_heuristic = []
        for j in range(len(grid[0])):
            h = goal[0] - i + goal[1] - j
            r_heuristic.append(h)
        
        heuristic.append(r_heuristic)
    # F = G + H
    x = init[0]
    y = init[1]
    expansion[x][y] = 0
    G = 0
    F = G + heuristic[x][y] 
    C = []
    C.append([F,G,x,y])
    count = 0

    found = False
    resign = False
    while found is False and resign is False:

        if len(C) == 0:
            resign = True
            print("Open Error")
        else:
            C.sort()
            C.reverse()
            next = C.pop()

            next_x = next[2]
            next_y = next[3]
            next_F = next[0]
            next_G = next[1]
            expansion[next_x][next_y] = count
            count += 1

            if next_x == goal[0] and next_y == goal[1]:
                found = True
                print("A Star searching Done..")
            else:
                for i in range(len(action)):
                    x2 = next_x + action[i][0]
                    y2 = next_y + action[i][1]
                    
                    if x2 >= 0  and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            G2 = next_G + 1
                            F2 = G2 + heuristic[x2][y2]

                            C.append([F2,G2,x2,y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i







                
