import numpy as np
from numpy import array
from filterpy.kalman import ExtendedKalmanFilter 
import math
import sympy
import yaml
from yaml.loader import SafeLoader
from Beacon_sim import Beacon_sim


def get_yaml_beacon(path = "/data/beacons.yaml"):
    print("get position of beacons")
    if not (path is None):
        with open(path, 'r') as f:
            beacons = yaml.load(f, Loader=SafeLoader)
            return beacons
    else:
        return None 

class Beacons_EKF():
    def __init__(self,init_position=[1.0,2.0,3.0], step_time = 0.2, q=5.0, r=0.2):
        self.EKF = ExtendedKalmanFilter(dim_x=6, dim_z=6)
        self.Beacons=[]
        self.Beacons , self.Beacons_num = self.read_beacon_position()
        self.dt = step_time
        self.Beacons_speed = [0.0,0.0,0.0]
        self.EKF.x = array([[ init_position[0], init_position[1], init_position[2], 0, 0, 0]]).T # x, y,z, vx,vy,vz
        ####--------
        self.H_def_jacbiano()
        self.EKF.F= np.array([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.EKF.Q = np.diag([1, 1, 1, 1, 1, 1])*(q**2)#ekf.P = np.diag([.1, .1, .1]) #Process noise matrix
        self.EKF.R = np.diag([1, 1, 1, 1, 1, 1])*(r**2)#Measurement noise matrix
        pass
        
    def H_def_jacbiano(self):
        x, y,z, x_vel,y_vel,z_vel  = sympy.symbols('x, y, z, x_vel,y_vel,z_vel')
        self.subs = {x: 0, y: 0, z:0, x_vel:0, y_vel:0,z_vel:0}
        self.px, self.py, self.pz, self.vx, self.vy, self.vz= x, y, z, x_vel,y_vel,z_vel  
        #
        equa_beacon1 = sympy.sqrt((x-self.Beacons[0]['x'])**2 + (y-self.Beacons[0]['y'])**2 + (z-self.Beacons[0]['z'])**2)
        equa_beacon2 = sympy.sqrt((x-self.Beacons[1]['x'])**2 + (y-self.Beacons[1]['y'])**2 + (z-self.Beacons[1]['z'])**2)
        equa_beacon3 = sympy.sqrt((x-self.Beacons[2]['x'])**2 + (y-self.Beacons[2]['y'])**2 + (z-self.Beacons[2]['z'])**2)

        equa_beacon_vel1 = sympy.sqrt((x-self.Beacons[0]['x']-self.dt*x_vel)**2 + (y-self.Beacons[0]['y']-self.dt*y_vel)**2 + (z-self.Beacons[0]['z']-self.dt*z_vel)**2)
        equa_beacon_vel2 = sympy.sqrt((x-self.Beacons[1]['x']-self.dt*x_vel)**2 + (y-self.Beacons[1]['y']-self.dt*y_vel)**2 + (z-self.Beacons[1]['z']-self.dt*z_vel)**2)
        equa_beacon_vel3 = sympy.sqrt((x-self.Beacons[2]['x']-self.dt*x_vel)**2 + (y-self.Beacons[2]['y']-self.dt*y_vel)**2 + (z-self.Beacons[2]['z']-self.dt*z_vel)**2)
        
        H = sympy.Matrix([[equa_beacon1],
                        [equa_beacon2],
                        [equa_beacon3],
                        [equa_beacon_vel1],
                        [equa_beacon_vel2],
                        [equa_beacon_vel3]])

        state = sympy.Matrix([ x, y, z, x_vel,y_vel,z_vel])
        self.H_j = H.jacobian(state)



    def read_beacon_position(self):
        #here read the values of beacon from beacons.yaml
        #and retun a list of beacon position
        Beacons_yaml = get_yaml_beacon(path = "./data/beacons.yaml")
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
        
        Beacons_dist.append(math.sqrt((x[0][0] - self.Beacons[0]['x'])**2+(x[1][0] - self.Beacons[0]['y'])**2+(x[2][0] - self.Beacons[0]['z'])**2))
        Beacons_dist.append(math.sqrt((x[0][0] - self.Beacons[1]['x'])**2+(x[1][0] - self.Beacons[1]['y'])**2+(x[2][0] - self.Beacons[1]['z'])**2))
        Beacons_dist.append(math.sqrt((x[0][0] - self.Beacons[2]['x'])**2+(x[1][0] - self.Beacons[2]['y'])**2+(x[2][0] - self.Beacons[2]['z'])**2))
        Beacons_dist.append(math.sqrt((x[0][0] -self.dt*x[3][0] - self.Beacons[0]['x'])**2+(x[1][0] -self.dt*x[4][0] - self.Beacons[0]['y'])**2+(x[2][0] -self.dt*x[5][0] - self.Beacons[0]['z'])**2))
        Beacons_dist.append(math.sqrt((x[0][0] -self.dt*x[3][0] - self.Beacons[1]['x'])**2+(x[1][0] -self.dt*x[4][0] - self.Beacons[1]['y'])**2+(x[2][0] -self.dt*x[5][0] - self.Beacons[1]['z'])**2))
        Beacons_dist.append(math.sqrt((x[0][0] -self.dt*x[3][0] - self.Beacons[2]['x'])**2+(x[1][0] -self.dt*x[4][0] - self.Beacons[2]['y'])**2+(x[2][0] -self.dt*x[5][0] - self.Beacons[2]['z'])**2))        
        return Beacons_dist


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
    EKF = Beacons_EKF(step_time=dt,init_position=[1.1,1.2,0.0])
    beacon = Beacon_sim(pos=[1.1,1.2,0.0],vel=[0.0,0.0,0.0],dt=dt)
    for i in range (int(20/dt)):
        z = beacon.update_position()
        EKF.update(z=z)
        EKF.predict()

    
