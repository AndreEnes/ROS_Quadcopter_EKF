import numpy as np
from numpy.random import randn
import math
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


class Beacon_sim():
    # this get 

    def __init__(self, dt=0.2, pos=[0,0,0], vel=[0,0,0]):
        self.pos = pos
        self.vel = vel
        self.Beacons_dist =[0,0,0]
        self.dt = dt
        self.Beacons , self.Beacons_num = self.read_beacon_position()
        print(self.Beacons)
        self.Beacons_dist= []
        self.Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[0]['x'])**2+(self.pos[1] - self.Beacons[0]['y'])**2+(self.pos[2] - self.Beacons[0]['z'])**2))
        self.Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[1]['x'])**2+(self.pos[1] - self.Beacons[1]['y'])**2+(self.pos[2] - self.Beacons[1]['z'])**2))
        self.Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[2]['x'])**2+(self.pos[1] - self.Beacons[2]['y'])**2+(self.pos[2] - self.Beacons[2]['z'])**2))
        self.Beacons_dist_last = self.Beacons_dist.copy()
        pass


    def read_beacon_position(self):
        #here read the values of beacon from beacons.yaml
        #and retun a list of beacon position
        Beacons_yaml = get_yaml_beacon(path = "./data/beacons.yaml")
        Beacons_num =Beacons_yaml['beacons_num']
        Beacons = []
        for c in range(0,Beacons_num) :
            Beacons.append(Beacons_yaml['beacons']['beacon'+str(c)])
        return (Beacons,Beacons_num)


    def update_position(self):
        print("update")

        self.vel[0] = self.vel[0]  + .1*randn()
        self.vel[1] = self.vel[1]  + .1*randn()
        self.vel[2] = self.vel[2]  + .1*randn()
        last_post = self.pos 
        self.pos[0] = self.pos[0] + self.vel[0]*self.dt
        self.pos[1] = self.pos[1] + self.vel[1]*self.dt
        self.pos[2] = self.pos[2] + self.vel[2]*self.dt

        self.Beacons_dist_last[0] = self.Beacons_dist[0]
        self.Beacons_dist_last[1] = self.Beacons_dist[1]
        self.Beacons_dist_last[2] = self.Beacons_dist[1]

        self.Beacons_dist[0] = math.sqrt((self.pos[0] - self.Beacons[0]['x'])**2+(self.pos[1] - self.Beacons[0]['y'])**2+(self.pos[2] - self.Beacons[0]['z'])**2)
        self.Beacons_dist[1] = math.sqrt((self.pos[0] - self.Beacons[1]['x'])**2+(self.pos[1] - self.Beacons[1]['y'])**2+(self.pos[2] - self.Beacons[1]['z'])**2)
        self.Beacons_dist[2] = math.sqrt((self.pos[0] - self.Beacons[2]['x'])**2+(self.pos[1] - self.Beacons[2]['y'])**2+(self.pos[2] - self.Beacons[2]['z'])**2)
        #Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[0]['x'])**2+(self.pos[1] - self.Beacons[0]['y'])**2+(self.pos[2] - self.Beacons[0]['z'])**2))
        #Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[1]['x'])**2+(self.pos[1] - self.Beacons[1]['y'])**2+(self.pos[2] - self.Beacons[1]['z'])**2))
        #Beacons_dist.append(math.sqrt((self.pos[0] - self.Beacons[2]['x'])**2+(self.pos[1] - self.Beacons[2]['y'])**2+(self.pos[2] - self.Beacons[2]['z'])**2))
        #speed = self.get_speed()
        #Beacons_dist.append(math.sqrt((self.pos[0] -self.dt*speed[0] - self.Beacons[0]['x'])**2+(self.pos[1] -self.dt*speed[1] - self.Beacons[0]['y'])**2+(self.pos[2] -self.dt*speed[2] - self.Beacons[0]['z'])**2))
        #Beacons_dist.append(math.sqrt((self.pos[0] -self.dt*speed[0] - self.Beacons[1]['x'])**2+(self.pos[1] -self.dt*speed[1] - self.Beacons[1]['y'])**2+(self.pos[2] -self.dt*speed[2] - self.Beacons[1]['z'])**2))
        #Beacons_dist.append(math.sqrt((self.pos[0] -self.dt*speed[0] - self.Beacons[2]['x'])**2+(self.pos[1] -self.dt*speed[1] - self.Beacons[2]['y'])**2+(self.pos[2] -self.dt*speed[2] - self.Beacons[2]['z'])**2))
        
        #return (self.pos,self.vel,self.Beacons_dist)
        #err = self.pos * 0.05*randn()
        #slant_dist = math.sqrt(self.pos**2 + self.alt**2)
        return np.array([[self.Beacons_dist[0],self.Beacons_dist[1],self.Beacons_dist[2],self.Beacons_dist_last[0],self.Beacons_dist_last[1],self.Beacons_dist_last[2]]]).T



if __name__ == "__main__":
    print("main")
    uno =Beacon_sim()
    print(uno.update_position())
    print(uno.update_position())
    print(uno.update_position())