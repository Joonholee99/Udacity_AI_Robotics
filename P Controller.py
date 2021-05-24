import random
import numpy as np
import matplotlib.pyplot as plt
import math
class Robot(object):
    def __init__(self, length = 20.0):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.length = length
        self.steering_noise = 0
        self.distance_noise = 0
        self.steering_drift = 0

    def set(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation%(2*np.pi)

    def set_noise(self, steering_noise, distance_noise):
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        self.steering_drift = drift

    def move(self, steering, distance, tolerance = 0.001, max_steering_angle = np.pi/4):
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0:
            distance = 0
        
        steer_input = random.gauss(steering, self.steering_noise)
        distance_input = random.gauss(distance, self.distance_noise)

        steer_input += self.steering_drift

        # Execute motion
        turn = np.tan(steer_input)/self.length * distance

        if abs(turn) < tolerance:
            self.x += distance * np.cos(self.orientation)
            self.y += distance * np.sin(self.orientation)
            self.orientation = (self.orientation + turn)%(2*np.pi)

        else:
            Radius = self.length / (np.tan(steer_input))
            cx = self.x - np.sin(self.orientation) * Radius
            cy = self.y + np.cos(self.orientation) * Radius
            self.orientation = (self.orientation + turn)%(2*np.pi)
            self.x = cx + np.sin(self.orientation) * Radius
            self.y = cy - np.cos(self.orientation) * Radius
            
    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


robot = Robot()
robot.set(0,1,0)
robot.set_steering_drift(10/180*np.pi)

def run(robot, params, n=100, speed=1):
    x_trajectory = []
    y_trajectory = []
    CTE_past = robot.y
    iCTE = 0
    sum_error = 0
    for i in range(n*2):
        CTE = robot.y - 0
        dCTE = CTE - CTE_past
        iCTE += CTE
        robot.move(steering = -params[0]*CTE - params[1]*dCTE - params[2]*iCTE, distance = speed)
        # print(robot.__repr__)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        CTE_past = CTE
        if i >= n:
            sum_error += CTE**2

    return x_trajectory, y_trajectory, sum_error/(n)

def twiddle(tol = 0.001):
    p = [0,0,0]
    dp = [1,1,1]
    robot = Robot()
    robot.set(0,1,0)
    robot.set_steering_drift(10/180*np.pi)
    x,y,best_error = run(robot,p)
    n=0
    while sum(dp) > tol:
        for i in range(3):
            p[i] += dp[i]
            robot = Robot()
            robot.set(0,1,0)
            robot.set_steering_drift(10/180*np.pi)
            x,y,error = run(robot,p)

            if error < best_error:
                best_error = error
                dp[i] *= 1.1
            else:
                p[i] -= 2*dp[i]
                robot = Robot()
                robot.set(0,1,0)
                robot.set_steering_drift(10/180*np.pi)
                x,y,other_error = run(robot,p)

                if other_error < best_error:
                    best_error = other_error
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        n += 1
        print(p)
        print("Error is",best_error)
        print("iter is",n)
        print("dp is",dp)
        print("--------------------------------")
                
    return p

p = twiddle()

robot = Robot()
robot.set(0,1,0)
robot.set_steering_drift(10/180*np.pi)
x_trajectory, y_trajectory, best_error = run(robot, p)
print(y_trajectory[-1])

n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2,1,figsize=(8,8))

ax1.plot(x_trajectory, y_trajectory, 'g', label = 'P Controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label = 'reference')