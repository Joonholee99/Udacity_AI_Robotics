import numpy as np
import math
import random
import copy
from math import *


class Plan:
    def __init__(self, grid, init, goal, cost=1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []

    # def make_heuristic(self, grid, goal, cost):
    #     self.heuristic = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    #     for i in range(len(grid)):
    #         r_heuristic = []
    #         for j in range(len(grid[0])):
    #             step = abs(goal[0] - i) + abs(goal[1] - j)
    #             r_heuristic.append(step)
            
    #         self.heuristic.append(r_heuristic)
    #     print(self.heuristic)

    #     return self.heuristic

    def make_heuristic(self, grid, goal, cost):
        self.heuristic = [[0 for row in range(len(grid[0]))] 
                            for col in range(len(grid))]
        for i in range(len(self.grid)):    
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + \
                    abs(j - self.goal[1])
        
        

    def astar(self):
        if self.heuristic == []:
            raise ValueError("Heuristic is empty!!")

        delta = [[-1,0],
                [1,0],
                [0,-1],
                [0,1]]
        closed = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid[0]))]
        action = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid[0]))]
        closed[self.init[0]][self.init[1]] = 1
        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g+h

        open = [[f,g,h,x,y]]
        found = False
        resign = False
        count = 0

        while found is False and resign is False:
            if len(open) == 0:
                resign = True
                print("Searching Done")
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]
                
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]

                    if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            h2 = self.heuristic[x2][y2]
                            g2 = g + self.cost
                            f2 = g2 + h2
                            closed[x2][y2] = 1
                            action[x2][y2] = i

                            open.append([f2,g2,h2,x2,y2])
                count += 1

        invpath = []
        invpath.append([self.goal[0],self.goal[1]])
        x = self.goal[0]
        y = self.goal[1]
        while x != self.init[0] or y != self.init[1]:
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            invpath.append([x2,y2])

            x = x2
            y = y2
        
        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])
        
        print(self.path)

    def smooth(self, weight_data = 0.1, weight_smooth = 0/1, tolerance = 0.00001):
        error = tolerance
        self.spath = copy.deepcopy(self.path)
        while error >= tolerance:
            error = 0
            for i in range(1,len(self.path)-1):
                for j in range(len(self.path[0])):
                    aux = self.spath[i][j]
                    self.spath[i][j] += weight_data * (self.path[i][j] - self.spath[i][j]) + weight_smooth * (self.spath[i+1][j] + self.spath[i-1][j] - 2 * self.spath[i][j])
                    
                    if i >= 2:
                        self.spath[i][j] += (1/2)* weight_smooth*(2*self.spath[i-1][j] - self.spath[i-2][j] - self.spath[i][j])
                    
                    if i <= len(self.path)-3:
                        self.spath[i][j] += (1/2) * weight_smooth* (2 * self.spath[i+1][j] - self.spath[i+2][j] - self.spath[i][j])
                    
                    error += abs(aux - self.spath[i][j])








class Robot:

    # --------
    # init: 
    #	creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length = 0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0
        self.num_collisions    = 0
        self.num_steps         = 0

    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)


    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise     = float(new_s_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    # --------
    # check: 
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = sqrt((self.x - float(i)) ** 2 + 
                                (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True
        
    def check_goal(self, goal, threshold = 1.0):
        dist =  sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold
        
    # --------
    # move: 
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, grid, steering, distance, 
             tolerance = 0.001, max_steering_angle = pi / 4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0


        # make a new copy
        res = Robot()
        res.length            = self.length
        res.steering_noise    = self.steering_noise
        res.distance_noise    = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions    = self.num_collisions
        res.num_steps         = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)


        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion

            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:

            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        # check for collision
        # res.check_collision(grid)

        return res

    # --------
    # sense: 
    #    

    def sense(self):

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    # 

    def measurement_prob(self, measurement):

        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        error *= exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))

        return error



    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]'  % (self.x, self.y)






# ------------------------------------------------
# 
# this is the particle filter class
#

class particles:

    # --------
    # init: 
    #	creates particle set with given initial position
    #

    def __init__(self, x, y, theta, 
                 steering_noise, distance_noise, measurement_noise, N = 100):
        self.N = N
        self.steering_noise    = steering_noise
        self.distance_noise    = distance_noise
        self.measurement_noise = measurement_noise
        
        self.data = []
        for i in range(self.N):
            r = Robot()
            r.set(x, y, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)


    # --------
    #
    # extract position from a particle set
    # 
    
    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.data[i].orientation
                              - self.data[0].orientation + pi) % (2.0 * pi)) 
                            + self.data[0].orientation - pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # --------
    #
    # motion of the particles
    # 

    def move(self, grid, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(grid, steer, speed)
            newdata.append(r)
        self.data = newdata

    # --------
    #
    # sensing and resampling
    # 

    def sense(self, Z):
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3

    



def run(grid, goal, spath, params, printflag = False, speed = 0.1, timeout = 1000):
    myrobot = Robot()
    myrobot.set(0,0,0)
    myrobot.set_noise(steering_noise, distance_noise, measurement_noise)
    filter = particles(myrobot.x, myrobot.y, myrobot.orientation, steering_noise, distance_noise, measurement_noise)
    cte = 0
    err = 0
    N = 0

    index = 0

    while not myrobot.check_goal(goal) and N < timeout:
        diff_cte = -cte

        estimate = filter.get_position()

        dx = spath[index+1][0] - spath[index][0]
        dy = spath[index+1][1] - spath[index][1]
        drx = estimate[0] - spath[index][0]
        dry = estimate[1] - spath[index][1]

        # u is the robot estimate projects onto the path segment
        u = (drx * dx + dry *dy) / (dx*dx + dy*dy)

        # cte is the estimate projected onto the normal of the path segment
        cte = (dry * dx - drx *dy)/(dx*dx + dy*dy)

        # pick the next path segment
        if u >1:
            index +=1

        diff_cte += cte
        steer = -params[0] * cte - params[1]*diff_cte
        
        myrobot = myrobot.move(grid, steer, speed)
        filter.move(grid, steer, speed)

        Z = myrobot.sense()
        filter.sense(Z)

        if not myrobot.check_collision(grid):
            print("#### Collision ####")

        err += (cte**2)
        N +=1

        if printflag:
            print(myrobot, cte, index, u)

    return [myrobot.check_goal(goal), myrobot.num_collisions, myrobot.num_steps]

def main(grid, init, goal, steering_noise, distance_noise, measurement_noise, weight_data, weight_smooth, p_gain, d_gain):
    path = Plan(grid, init, goal)
    print("Planning Done...")
    path.astar()
    print("A star Done...")
    print(path.spath)
    path.smooth(weight_data, weight_smooth)
    print("Smoothing Done")
    return run(grid, goal, path.spath, [p_gain, d_gain])



grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 0]]


init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]


steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3

weight_data       = 0.1
weight_smooth     = 0.2
p_gain            = 2.0
d_gain            = 6.0

    
print(main(grid, init, goal, steering_noise, distance_noise, measurement_noise, 
           weight_data, weight_smooth, p_gain, d_gain))







    

# def Astar_search(grid, init, goal):

#     closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
#     closed[init[0]][init[1]] = 1

#     expansion = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
#     action = [[1,0],
#             [-1,0],
#             [0,1],
#             [0,-1]]
#     action_p = ['V','^','>','<']
#     heuristic = []

#     for i in range(len(grid)):
#         r_heuristic = []
#         for j in range(len(grid[0])):
#             h = goal[0] - i + goal[1] - j
#             r_heuristic.append(h)
        
#         heuristic.append(r_heuristic)
#     # F = G + H
#     x = init[0]
#     y = init[1]
#     expansion[x][y] = 0
#     G = 0
#     F = G + heuristic[x][y] 
#     C = []
#     C.append([F,G,x,y])
#     count = 0

#     found = False
#     resign = False
#     while found is False and resign is False:

#         if len(C) == 0:
#             resign = True
#             print("Open Error")
#         else:
#             C.sort()
#             C.reverse()
#             next = C.pop()

#             next_x = next[2]
#             next_y = next[3]
#             next_F = next[0]
#             next_G = next[1]
#             expansion[next_x][next_y] = count
#             count += 1

#             if next_x == goal[0] and next_y == goal[1]:
#                 found = True
#                 print("A Star searching Done..")
#             else:
#                 for i in range(len(action)):
#                     x2 = next_x + action[i][0]
#                     y2 = next_y + action[i][1]
                    
#                     if x2 >= 0  and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
#                         if closed[x2][y2] == 0 and grid[x2][y2] == 0:
#                             G2 = next_G + 1
#                             F2 = G2 + heuristic[x2][y2]

#                             C.append([F2,G2,x2,y2])
#                             closed[x2][y2] = 1
#                             action[x2][y2] = i







                
