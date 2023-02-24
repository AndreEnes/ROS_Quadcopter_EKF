import numpy as np
from numpy import array,asarray
from filterpy.kalman import ExtendedKalmanFilter 
import math
import sympy
import yaml
from yaml.loader import SafeLoader
from Beacon_sim import Beacon_sim
import matplotlib.pyplot as plt


def get_yaml_beacon(path = "../data/beacons.yaml"):
    #print("get position of beacons")
    if not (path is None):
        with open(path, 'r') as f:
            beacons = yaml.load(f, Loader=SafeLoader)
            return beacons
    else:
        return None 

class Beacons_EKF():
    def __init__(self,init_position=[1.0,2.0,3.0],init_speed=[1.0,2.0,3.0], step_time = 0.2, q=5.0, r=0.2):
        self.EKF = ExtendedKalmanFilter(dim_x=6, dim_z=6)
        self.Beacons=[]
        self.Beacons , self.Beacons_num = self.read_beacon_position()
        self.dt = step_time
        self.EKF.x = array([[ init_position[0], init_position[1], init_position[2], init_speed[0], init_speed[1], init_speed[2]]]).T # x, y,z, vx,vy,vz
        self.H_def_jacbiano()
        self.EKF.F= np.array([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.EKF.Q = np.diag([1, 1, 1, 1, 1, 1])*(q**2)#ekf.P = np.diag([.1, .1, .1]) #Process noise matrix

        self.EKF.R = np.eye(self.Beacons_num*2)*(r**2)#Measurement noise matrix
        #print(self.EKF.R)
        pass

    def get_beacon_point(self):
        beacon_point = []
        for i in range(self.Beacons_num):
            beacon_point.append([self.Beacons[i]['x'],self.Beacons[i]['y'],self.Beacons[i]['z']])
        return beacon_point
        
    def H_def_jacbiano(self):
        x, y,z, x_vel,y_vel,z_vel  = sympy.symbols('x, y, z, x_vel,y_vel,z_vel')
        self.subs = {x: 0, y: 0, z:0, x_vel:0, y_vel:0,z_vel:0}
        self.px, self.py, self.pz, self.vx, self.vy, self.vz= x, y, z, x_vel,y_vel,z_vel  
        H_matriz =  []

        for i in range(self.Beacons_num):
            H_matriz.append([sympy.sqrt((x-self.Beacons[i]['x'])**2 + (y-self.Beacons[i]['y'])**2 + (z-self.Beacons[i]['z'])**2)])
        
        for i in range(self.Beacons_num):
            H_matriz.append([sympy.sqrt((x-self.Beacons[i]['x']-self.dt*x_vel)**2 + (y-self.Beacons[i]['y']-self.dt*y_vel)**2 + (z-self.Beacons[i]['z']-self.dt*z_vel)**2)])
        H = sympy.Matrix(H_matriz)

        state = sympy.Matrix([ x, y, z, x_vel,y_vel,z_vel])
        self.H_j = H.jacobian(state)

        pass



    def read_beacon_position(self):
        #here read the values of beacon from beacons.yaml
        #and retun a list of beacon position
        Beacons_yaml = get_yaml_beacon(path = "../data/beacons.yaml")
        Beacons_num =Beacons_yaml['beacons_num']
        Beacons = []
        for c in range(0,Beacons_num) :
            Beacons.append(Beacons_yaml['beacons']['beacon'+str(c)])
        return (Beacons,Beacons_num)

    def HJacobian_at(self,x):
        """ compute Jacobian of H matrix where h(x) computes the range and
        bearing to a landmark for state x """
        self.subs[self.px] = x[0][0]
        self.subs[self.py] = x[1][0]
        self.subs[self.pz] = x[2][0]
        self.subs[self.vx] = x[3][0]
        self.subs[self.vy] = x[4][0]
        self.subs[self.vz] = x[5][0]

        H = array(self.H_j.evalf(subs=self.subs)).astype(float)
        return H

    def hx(self,x):
        """ compute measurement for slant range that would correspond 
        to state x.
        """
        Beacons_dist= []

        for i in range(self.Beacons_num):
            Beacons_dist.append(math.sqrt((x[0] - self.Beacons[i]['x'])**2+(x[1] - self.Beacons[i]['y'])**2+(x[2]- self.Beacons[i]['z'])**2))

        for i in range(self.Beacons_num):
            Beacons_dist.append(math.sqrt((x[0] -self.dt*x[3] - self.Beacons[i]['x'])**2+(x[1]-self.dt*x[4] - self.Beacons[i]['y'])**2+(x[2] -self.dt*x[5]- self.Beacons[i]['z'])**2))
      
        return array([Beacons_dist]).T


    def residual(self,a, b):
        """ compute residual (a-b) between measurements containing 
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

    def get_speed(self):
        return[0,0,0]


    def update(self,z):
        self.EKF.update(z, self.HJacobian_at, self.hx)


    def predict(self):
        self.EKF.predict()

    def get_status(self):
        return self.EKF.x

    def get_sensor_reading():   # get beacon reading 
        return np.random.normal(130, 3, 1)  # 1st number = mu // 2nd number = sigma // 3rd number = no. of samples


if __name__ == '__main__':
    dt = 0.2
    xs, track_pos = [], []
    xs1 ,xs2, xs3= [],[],[]
    track1,track2 = [],[]
    EKF = Beacons_EKF(step_time=dt,init_position=[1.1,1.2,0.0],init_speed=[1.0,2.0,3.0])
    beacon = Beacon_sim(pos=[1.1,1.2,0.0],vel=[10.0,10.0,0.0],dt=dt)
    for i in range (int(20/dt)):
        z = beacon.update_position()
        track_pos.append(beacon.get_pos().copy())#, beacon.get_vel(), z].copy())
        EKF.update(z=z)
        xs.append(EKF.get_status())
        EKF.predict()
        print(str(beacon.get_pos())+"--"+str(EKF.get_status()))
    time = np.arange(0, len(xs)*dt, dt)
    xs1 ,xs2, xs3= [],[],[]
    for i in range(len(xs)):
        xs1.append(xs[i][0][0])
        xs2.append(xs[i][1][0])
        xs3.append(xs[i][2][0])
        #track1.append(track[i][0][0])
        #print(track_pos[i][0][0])
    plt.plot(xs1, time, label = "line 1")
    plt.plot(track1, time, label = "line 2")
    #print("track")
    #print(track[-1][0][0])

    plt.ylabel('some numbers')

    plt.show()