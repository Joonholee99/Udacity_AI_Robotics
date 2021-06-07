import numpy as np
import math
import random
import copy


class Plan:
    def __init__(self, grid, init, goal, cost=1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []

    def make_heuristic(self, grid, goal, cost):
        heuristic = []
        for i in range(len(grid)):
            r_heuristic = []
            for j in range(len(grid[0])):
                step = abs(goal[0] - i) + abs(goal[1] - j)
                r_heuristic.append(step)
            
            heuristic.append(r_heuristic)

        return heuristic

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
                    x2 = x + delta[0]
                    y2 = y + delta[1]

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
        while x != self.init[0] and y != self.init[1]:
            x2 = x - action[x][y]
            y2 = y - action[x][y]
            invpath.append([x2,y2])

            x = x2
            y = y2
        
        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])

    def smooth(self, weight_data = 0.1, weight_smooth = 0/1, tolerance = 0.00001):
        error = tolerance
        self.spath = copy.deepcopy(self.path)
        while error >= tolerance:
            error = 0
            for i in range(1,len(self.path)):
                for j in range(self.path[0]):
                    aux = self.spath[i][j]
                    self.spath += weight_data * (self.path[i][j] - self.spath[i][j]) + weight_smooth * (self.spath[i+1][j] + self.spath[i-1][j])
                    
                    if i >= 2:
                        self.spath[i][j] += (1/2)* weight_smooth*(2*self.spath[i-1][j] - self.spath[i-2][j] - self.spath[i][j])
                    
                    if i < len(self.path)-3:
                        self.spath[i][j] += (1/2) * weight_smooth* (2 * self.spath[i+1][j] - self.spath[i+2][j] - self.spath[i][j])
                    
                    error += abs(aux - self.spath[i][j])








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

class particles:

    def __init__(self, x, y, theta, steering_noise, distance_noise, measurement_noise, N=100):
        self.N = N
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

        self.data = []
        for i in range(self.N):
            r = Robot()
            r.set(x,y,theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)

    def get_position(self):
        x = 0
        y = 0
        orientation = 0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            orientation += (((self.data[i].orientation
                              - self.data[0].orientation + math.pi) % (2.0 * math.pi)) 
                            + self.data[0].orientation - math.pi)
            
        return [x/self.N, y/self.N, orientation/self.N]

    def move(self, grid, steer, speed):
        newdata=[]

        for i in range(self.N):
            r = self.data[i].move(grid, steer, speed)
            newdata.append(r)
        
        self.data = newdata

    def sense(self, Z):
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

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
        drx = estimate






    

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







                
