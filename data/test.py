import numpy as np
from filterpy.kalman import KalmanFilter

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import eye, array, asarray,dot,sqrt
import numpy as np
import sympy
import yaml
from yaml.loader import SafeLoader


def get_yaml_beacon(path = "/data/beacons.yaml"):
    print("obtener las misiones para ser usadas y comparadas con la que envian")
    if not (path is None):
        with open(path, 'r') as f:
            beacons = yaml.load(f, Loader=SafeLoader)
            return beacons
    else:
        return None 

def H_of(x, landmark_pos):
    """ compute Jacobian of H matrix where h(x) computes the range and
    bearing to a landmark for state x """

    px = landmark_pos[0]
    py = landmark_pos[1]
    hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
    dist = sqrt(hyp)

    H = array(
        [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
         [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    return H


def hx(x):
    """ compute measurement for slant range that would correspond 
    to state x.
    """
    
    return (x[0]**2 + x[2]**2) ** 0.5

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y


class Beacons_EKF(EKF):
    def __init__(self, step_time = 0.2, q=5.0, r=0.2):
        EKF.__init__(self, 6, 6,0)
        self.Beacons=[]
        self.Beacons , self.Beacons_num = self.read_beacon_position()
        self.dt = step_time

        ####--------
        x, y,z, x_vel,y_vel,z_vel  = sympy.symbols('x, y, z, x_vel,y_vel,z_vel')
        self.subs = {x: 0, y: 0, z:0, x_vel:0, y_vel:0,z_vel:0}
        self.px, self.py, self.pz, self.vx, self.vy, self.vz= x, y, z, x_vel,y_vel,z_vel  
        #
        equa_beacon1 = sympy.sqrt((x-self.Beacons[0][0])**2 + (y-self.Beacons[0][1])**2 + (z-self.Beacons[0][2])**2)
        equa_beacon2 = sympy.sqrt((x-self.Beacons[1][0])**2 + (y-self.Beacons[1][1])**2 + (z-self.Beacons[1][2])**2)
        equa_beacon3 = sympy.sqrt((x-self.Beacons[2][0])**2 + (y-self.Beacons[2][1])**2 + (z-self.Beacons[2][2])**2)

        equa_beacon_vel1 = sympy.sqrt((x-self.Beacons[0][0]-self.dt*x_vel)**2 + (y-self.Beacons[0][1]-self.dt*y_vel)**2 + (z-self.Beacons[0][2]-self.dt*z_vel)**2)
        equa_beacon_vel2 = sympy.sqrt((x-self.Beacons[1][0]-self.dt*x_vel)**2 + (y-self.Beacons[1][1]-self.dt*y_vel)**2 + (z-self.Beacons[1][2]-self.dt*z_vel)**2)
        equa_beacon_vel3 = sympy.sqrt((x-self.Beacons[2][0]-self.dt*x_vel)**2 + (y-self.Beacons[2][1]-self.dt*y_vel)**2 + (z-self.Beacons[2][2]-self.dt*z_vel)**2)
        H = sympy.Matrix([[equa_beacon1],
                        [equa_beacon2],
                        [equa_beacon3],
                        [equa_beacon_vel1],
                        [equa_beacon_vel2],
                        [equa_beacon_vel3]])
        state = sympy.Matrix([ x, y, z, x_vel,y_vel,z_vel])
        J = H.jacobian(state)


        ####-----F = A +B
        self.x = array([[ 0, 0, 0, 0, 0, 0]]).T # x, y,z, vx,vy,vz
        self.F= np.array([[1, 0, 0, self.T, 0, 0],
                            [0, 1, 0, 0, self.T, 0],
                            [0, 0, 1, 0, 0, self.T],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.Q = np.ones(6)*(q**2)#ekf.P = np.diag([.1, .1, .1]) #Process noise matrix
        self.R = np.ones(self.Beacons_num*2)*(r**2)#Measurement noise matrix
        self.P *= 50
        self.Beacons_speed = [0,0,0]



    def read_beacon_position(self):
        #here read the values of beacon from beacons.yaml
        #and retun a list of beacon position
        Beacons_yaml = get_yaml_beacon(path = "/data/beacons.yaml")
        Beacons_num =Beacons_yaml['beacons_num']
        Beacons = []
        for c in range(0,self.Beacons_num) :
            Beacons.append(Beacons_yaml['beacons']['beacon'+str(c)])
        return (Beacons,Beacons_num)

    def predict(self,position,speed):
        self.subs[self.px] = position[0]
        self.subs[self.py] = position[1]
        self.subs[self.pz] = position[2]
        self.subs[self.vx] = speed[0]
        self.subs[self.vy] = speed[1]
        self.subs[self.vz] = speed[2]
        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        #self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)

        self.P = F @ self.P @ F.T 

            




    def get_sensor_reading():   # get beacon reading 

        return np.random.normal(130, 3, 1)  # 1st number = mu // 2nd number = sigma // 3rd number = no. of samples


if __name__ == '__main__':
    hola = Beacons_EKF()

    
