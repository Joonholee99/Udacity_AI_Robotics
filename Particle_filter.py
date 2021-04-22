import numpy as np
import math
import random

landmarks = [[20,20],[80,80],[20,80],[80,20]]
world_size = 100
pi = 3.1415

# print(landmarks[2][1])


def resampling(w):
    norm_coeff = sum(w)
    w_array = np.array(w)
    w_norm = w_array/norm_coeff
    
    return random.choices(range(len(w)),weights=w_norm,k=len(w))

def error(r,p):
    err = 0
    for i in range(1000):
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        
        err += math.sqrt(dx**2 + dy**2)

    return err/1000
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orient = random.random() * pi *2

        self.forward_noise = 0
        self.turn_noise = 0
        self.sense_noise = 0


    def set(self, new_x, new_y, new_orient):
        
        if new_x < 0 or new_x > world_size:
            raise ValueError("X coordinate out of bound")

        elif new_y < 0 or new_y > world_size:
            raise ValueError("Y coordinate out of bound")

        elif new_orient < 0 or new_orient >=2*pi:
            raise ValueError("orient coordinate out of bound")

        self.x = new_x
        self.y = new_y
        self.orient = new_orient

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        self.forward_noise = new_f_noise
        self.turn_noise = new_t_noise
        self.sense_noise = new_s_noise

    def sense(self):
        a = []
        for i in range(len(landmarks)):
            distance = math.sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            distance += random.gauss(0,self.sense_noise)
            a.append(distance)
        return a

    def move(self, turn, forward):
        turn += random.gauss(0,self.turn_noise)
        forward += random.gauss(0,self.forward_noise)

        orient = (self.orient+turn)%(2*pi)
        x = (self.x + forward*math.cos(self.orient))%world_size
        y = (self.y + forward*math.sin(self.orient))%world_size

        res = robot()
        res.set(x,y,orient)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        return res

    def Gaussian(self,mu, sigma, x):

        return np.exp(-(x - mu)**2/(2*sigma**2))/(math.sqrt(2*pi*sigma**2))

    def measurement_prob(self, measurement):
        prob = 1

        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            prob *= self.Gaussian(dist, self.sense_noise,measurement[i])

        return prob

myrobot = robot()
p = []

N = 1000
for i in range(N):
    x = robot()
    x.set_noise(0.05,0.05,5.0)
    p.append(x)
print(error(myrobot,p))
for j in range(10):
    
    myrobot = myrobot.move(0.1,5)
    Z = myrobot.sense()

    p2 =[]
    for i in range(N):
        p2.append(p[i].move(0.1,5))

    p = p2

    w = []

    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    p3 = []
    resample = resampling(w)

    for i in range(N):
        p3.append(p[resample[i]])

    p = p3
    print(error(myrobot,p))
# print("--------------------------------------\n")
# for i in range(N):
    
    
#     print(p[i].x)

# print("--------------------------------------\n")
# for i in range(N):
    
    
#     print(p[i].y)

    