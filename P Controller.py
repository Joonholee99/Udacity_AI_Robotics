import random
import numpy as np
import matplotlib.pyplot as plt

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

def run(robot, tau_p, tau_d,tau_i, n=100, speed=1):
    x_trajectory = []
    y_trajectory = []
    CTE_past = robot.y
    sum_error = 0
    for i in range(n):
        CTE = robot.y - 0
        dCTE = CTE - CTE_past
        sum_error += CTE
        robot.move(steering = -tau_p*CTE - tau_d*dCTE - tau_i*sum_error, distance = speed)
        print(robot.__repr__)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        CTE_past = CTE

    return x_trajectory, y_trajectory

x_trajectory, y_trajectory = run(robot, 0.2,3,0.004)

n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2,1,figsize=(8,8))

ax1.plot(x_trajectory, y_trajectory, 'g', label = 'P Controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label = 'reference')