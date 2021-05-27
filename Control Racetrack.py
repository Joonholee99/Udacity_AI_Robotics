import numpy as np
import math
import random

class robot:
    def __init__(self,length = 20):
        robot.x = 0
        robot.y = 0
        robot.orientation = 0
        robot.length = length
        robot.steering_noise = 0
        robot.distance_noise = 0
        robot.steering_drift = 0
    
    def set(self, x,y,orientation):
        robot.x = x
        robot.y = y
        robot.orientation = orientation

    def set_noise(self, steering_noise, distance_noise):
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, steering_drift):
        self.steering_drift = steering_drift

    def move(self, steering, distance, tol = 0.001, max_steering_angle = math.pi/4):
        if steering >= max_steering_angle:
            steering = max_steering_angle
        elif steering <= -max_steering_angle:
            steering = -max_steering_angle
        
        if distance<0:
            distance = 0
        
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        steering2 += self.steering_drift

        turn = math.tan(steering2) * distance2/(self.length)

        if abs(turn) < tol:
            self.x += (distance2 * math.cos(self.orientation))
            self.y += (distance2 * math.sin(self.orientation))
            self.orientation = (self.orientation + turn)%(math.pi * 2)

        else:
            radius = distance2 / turn
            cx = self.x - (math.sin(self.orientation) * radius)
            cy = self.y + (math.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * math.pi)
            self.x = cx + (math.sin(self.orientation) * radius)
            self.y = cy - (math.cos(self.orientation) * radius)


def track(radius, robot):
    x = robot.x
    if ((robot.orientation >= 0) and robot.orientation <= (0.5*math.pi)) or ((robot.orientation >= 1.5*math.pi) and (robot.orientation <= 2*math.pi)):
        if x < radius and x >= 0:
            y = math.sqrt(radius**2 - (x - radius)**2)
        elif x >= radius and x < 3*radius:
            y = radius
        elif x >= 3*radius and x <= 4*radius:
            y = math.sqrt(radius**2 - (x - 3*radius)**2)
        else:
            y = 0

    elif ((robot.orientation >= 0.5*math.pi) and robot.orientation <= (1.5*math.pi)):
        if x < radius and x >= 0:
            y = -math.sqrt(radius**2 - (x - radius)**2)
        elif x >= radius and x < 3*radius:
            y = -radius
        elif x >= 3*radius and x <= 4*radius:
            y = -math.sqrt(radius**2 - (x - 3*radius)**2)
        else:
            y = 0

    if robot.y >= 0:
        cte = robot.y - y
    else:
        cte = y - robot.y

    return cte

def run(params, radius, printflag = False):
    myrobot = robot()
    myrobot.set(0,0,math.pi/2)
    speed = 1
    err = 0
    int_crosstrack_error = 0
    N = 200

    crosstrack_error = track(radius, myrobot)

    for i in range(2*N):
        diff_crosstrack_error = -crosstrack_error
        crosstrack_error = track(radius, myrobot)
        diff_crosstrack_error += crosstrack_error
        int_crosstrack_error += crosstrack_error

        steer = - params[0] * crosstrack_error\
                -params[1] * diff_crosstrack_error\
                -params[2] * int_crosstrack_error

        myrobot.move(steer, speed)
        if i >= N:
            err += crosstrack_error**2
        if printflag:
            print(myrobot.x,myrobot.y)

    return err / N

radius = 25
params = [10,15,0.1]
err = run(params, radius, True)

