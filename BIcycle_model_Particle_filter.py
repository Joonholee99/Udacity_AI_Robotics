import math
import numpy as np
import random

pi = 3.14
landmarks = [[0,100],[0,0],[100,0],[100,100]]
world_size = 100
max_steering_angle = pi/4

bearing_noise = 0.1
steering_noise = 0.1
distance_noise = 5
sense_noise = 0
length = 20

class Robot:
    def __init__(self,length=20):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * pi*2

        self.length = length
        self.bearing_noise = 0
        self.steering_noise = 0
        self.sense_noise = 0
        self.distance_noise = 0

    def set(self, new_x, new_y, new_orientation):
        self.x = new_x
        self.y = new_y
        self.orientation = new_orientation

    def set_noise(self, n_bearing_noise, n_steering_noise, n_sense_noise,n_distance_noise):
        self.bearing_noise = n_bearing_noise
        self.steering_noise = n_steering_noise
        self.sense_noise = n_sense_noise
        self.distance_noise = n_distance_noise

    def move(self, motion):
        #motion = [steering, distance]
        steering=0
        distance=0
        steering += motion[0] + random.gauss(0,self.steering_noise)
        distance += motion[1] + random.gauss(0,self.distance_noise)
        
        turn_angle = distance / self.length * math.tan(steering)
        if abs(turn_angle) > 0.001:    
            R = distance/turn_angle
        
            new_x = self.x + R*(math.sin(self.orientation+turn_angle) - math.sin(self.orientation))
            new_y = self.y + R*(math.cos(self.orientation) - math.cos(self.orientation+turn_angle))
            new_orientation = (self.orientation + turn_angle)%(2*pi)
        else:    
            new_x = self.x + distance * math.cos(self.orientation)
            new_y = self.y + distance * math.sin(self.orientation)
            new_orientation = self.orientation
        res = Robot()
        res.set(new_x,new_y,new_orientation)
        res.set_noise(self.bearing_noise,self.steering_noise,self.sense_noise,self.distance_noise)
        return res
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dx = landmarks[i][1] - self.x
            dy = landmarks[i][0] - self.y
            dtheta = math.atan2(dy,dx) - self.orientation + random.gauss(0,self.sense_noise)
            if dtheta < 0:
                dtheta = dtheta + 2*pi
            Z.append(dtheta)
        return Z 

    def measurement_prob(self, Z):
        predicted_measurements = self.sense()
        error = 1

        for i in range(len(Z)):
            error_bearing = abs(Z[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            

            # update Gaussian
            error *= (np.exp(- (error_bearing ** 2) / (self.bearing_noise ** 2) / 2.0) /  
                      math.sqrt(2.0 * pi * (self.bearing_noise ** 2)))

        return error


def resampling(weight):

    norm_coeff = sum(weight)
    if norm_coeff == 0:
        raise ValueError
        
    w = []

    for i in range(len(weight)):
        w.append(weight[i]/norm_coeff)
    
    
    return random.choices(list(range(len(weight))),weights=w,k=len(weight))

def get_position(p):
    x = 0
    y = 0
    orientation = 0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        orientation  += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)

    return [x/len(p),y/len(p),orientation/len(p)]

def generate_ground_truth(motions):
    myrobot = Robot()
    myrobot.set_noise(bearing_noise,steering_noise,sense_noise,distance_noise)

    Z = []

    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())

    return [myrobot, Z]


def particle_filter(motions, measurement, N=500):
    p = []

    for i in range(N):
        p_s = Robot()
        p_s.set_noise(bearing_noise,steering_noise,sense_noise,distance_noise)
        p.append(p_s)

    for i in range(len(motions)):
        p2 = []

        for j in range(N):
            p_s = p[j].move(motions[i])
            p2.append(p_s)

        p = p2

        w = []
        for j in range(N):
            prob = p[j].measurement_prob(measurement[i])

            w.append(prob)

        index = resampling(w)
        p3=[]
        for j in range(N):
            sample_index = index[j]
            p3.append(p[sample_index])

        p = p3

    return get_position(p)

iter = 10

motions_in = [[2*pi/20,12] for row in range(iter)]
x = generate_ground_truth(motions_in)
final_robot = x[0]
measurements_in = x[1]
estimated_position = particle_filter(motions_in,measurements_in,N=500)
print(final_robot.x,final_robot.y,final_robot.orientation)
print(estimated_position)