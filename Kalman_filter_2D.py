import numpy as np 
from numpy.linalg import inv

measurements = np.array([[1],[2],[3]])
# print(measurements[0].shape)
x = np.array([[0],[0]])
# print(x.shape)
P = np.array([[1000,0],[0,1000]])
u = np.array([[0],[0]])
F = np.array([[1, 1],[0 ,1]])
H = np.array([[1, 0]])
# print(H.shape)
# print(H*x)
R = np.array([[1]])
I = np.array([[1, 0],[0, 1]])

# print(np.identity(2))

def predict(x,P):
    x_dot = np.matmul(F,x) + u

    P_dot = np.matmul(np.matmul(F,P),np.transpose(F))
    return [x_dot, P_dot]

def measurements_update(x,P,measurement):
    y = measurement - np.matmul(H,x)
    print(y.shape)
    S = np.matmul(np.matmul(H,P),np.transpose(H)) + R
    K = np.matmul(np.matmul(P,np.transpose(H)),inv(S))

    x_update = x + np.matmul(K,y)
    P_update = np.matmul((I - np.matmul(K,H)),P)
    # print(x_update.shape)
    return [x_update, P_update]



def filter(x, P,measurements):
    
    for i in range(len(measurements)):
        [x, P] = measurements_update(x,P,measurements[i])
        # print("update X = \n",x)
        # print("update P = \n",P)
        [x, P] = predict(x,P)
        print("predict X = \n",x)
        print("predict P = \n",P)

filter(x,P,measurements)
