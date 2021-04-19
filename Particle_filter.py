import numpy as np
import math
import random

landmarks = [[20,20],[80,80],[20,80],[80,20]]
world_size = 100
pi = 3.1415

# print(landmarks[2][1])

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
            raise ValueError, ' X coordinate out of bound '

        elif new_y < 0 or new_y > world_size:
            raise ValueError, ' Y coordinate out of bound '

        elif new_orient < 0 or new_orient >=2*pi:
            raise ValueError, ' orient coordinate out of bound '

        self.x = new_x
        self.y = new_y
        self.orient = new_orient

    def sense(self):
        a = []
        for i in range(len(landmarks)):
            distance = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            distance += random.gauss(0,self.sense_noise)
            a.append(distance)
        return a

    def move(self, turn, forward):
        
        