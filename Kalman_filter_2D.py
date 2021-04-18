import numpy as np 
from numpy.linalg import inv

measurements = np.array([[5, 10],[6, 8],[7, 6],[8, 4],[9,2],[10,0]])
print(len(measurements))
x = np.array([[4],[12],[0],[0]]) # state = [x,y,x_dot,y_dot]
P = np.diag([0,0,1000,1000]) #zeros uncertainty for Position and 1000 uncertainty for vel
u = np.array([[0],[0],[0],[0]]) #no external input
dt = 0.1
F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
H = np.array([[1,0,0,0],[0,1,0,0]])

R = np.diag([0.1, 0.1])
I = np.identity(4)


def predict(x,P):
    x_dot = np.matmul(F,x) + u

    P_dot = np.matmul(np.matmul(F,P),np.transpose(F))
    return [x_dot, P_dot]

def measurements_update(x,P,measurement):
    y = np.reshape(measurement,(2,1)) - np.matmul(H,x)
    # print(np.reshape(measurement,(2,1)))
    # print(y)
    S = np.matmul(np.matmul(H,P),np.transpose(H)) + R
    K = np.matmul(np.matmul(P,np.transpose(H)),inv(S))

    x_update = x + np.matmul(K,y)
    P_update = np.matmul((I - np.matmul(K,H)),P)

    return [x_update, P_update]



def filter(x, P,measurements):
    
    for i in range(len(measurements)):
        

        [x, P] = predict(x,P)
        [x, P] = measurements_update(x,P,measurements[i])
        print("predict X = \n",x)
        print("predict P = \n",P)

filter(x,P,measurements)
