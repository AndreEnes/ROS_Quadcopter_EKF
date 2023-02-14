import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

def get_sensor_reading():   # get beacon reading 
    return np.random.normal(130, 3, 1)  # 1st number = mu // 2nd number = sigma // 3rd number = no. of samples

# initialize KalmanFilter

#no control
T = 2
q = 3

x_k = 1
y_k = 2
z_k = 3
Vx_k = 0
Vy_k = 0
Vz_k = 0


u_k = 0

A = np.array([[1, 0, 0, T, 0, 0],
             [0, 1, 0, 0, T, 0],
             [0, 0, 1, 0, 0, T],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]])

W_k = np.array([[q**2, 0, 0, 0, 0, 0],
                [0, q**2, 0, 0, 0, 0],
                [0, 0, q**2, 0, 0, 0],
                [0, 0, 0, q**2, 0, 0],
                [0, 0, 0, 0, q**2, 0],
                [0, 0, 0, 0, 0, q**2]])

z_k = np.array([[],
                [],
                [],
                [],
                [],
                []])
# Run loop
'''''
for i in range(100):
    z = get_sensor_reading()
    f.predict()
    f.update(z)
    print(f.x)'''