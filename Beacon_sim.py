import numpy as np
from numpy.random import randn
import math
import yaml
from yaml.loader import SafeLoader

#def square_trayect(size=[50,50,50]):



def get_yaml_beacon(path = "/data/beacons.yaml"):
    #print("obtener las misiones para ser usadas y comparadas con la que envian")
    if not (path is None):
        with open(path, 'r') as f:
            beacons = yaml.load(f, Loader=SafeLoader)
            return beacons
    else:
        return None 


class Beacon_sim():
    # this get 

    def __init__(self, dt=0.2, pos=[0.0,0.0,0.0], vel=[0.0,0.0,0.0],ruido=1):
        # ruido  es un factor que multiplica el ruido condistribucion gausiana
        self.def_trayec = False
        self.trayec = []
        self.pos = pos
        self.vel = vel
        self.velcal =vel
        self.count = 0
        self.Beacons_dist =[0,0,0]
        self.dt = dt
        self.ruido= ruido
        self.Beacons , self.Beacons_num = self.read_beacon_position()
        #print(self.Beacons)
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
        #print("update")
        self.count=self.count +1
        pos_last =[]
        pos_last.append(self.pos[0])
        pos_last.append(self.pos[1])
        pos_last.append(self.pos[2])

        if (self.def_trayec):
            self.pos[0] = self.trayec[0][self.count]
            self.pos[1] = self.trayec[1][self.count]
            self.pos[2] = self.trayec[2][self.count]
        else:
            self.vel[0] = self.vel[0]  + 0.1*randn()
            self.vel[1] = self.vel[1]  + 0.1*randn()
            self.vel[2] = self.vel[2]  + 0.1*randn()
            self.pos[0] = self.pos[0] + self.vel[0]*self.dt
            self.pos[1] = self.pos[1] + self.vel[1]*self.dt
            self.pos[2] = self.pos[2] + self.vel[2]*self.dt
            

        self.velcal[0] =  (self.pos[0] -pos_last[0])/self.dt
        self.velcal[1] =  (self.pos[1] -pos_last[1])/self.dt
        self.velcal[2] =  (self.pos[2] -pos_last[2])/self.dt


        self.Beacons_dist_last[0] = self.Beacons_dist[0]
        self.Beacons_dist_last[1] = self.Beacons_dist[1]
        self.Beacons_dist_last[2] = self.Beacons_dist[2]

        self.Beacons_dist[0] = math.sqrt((self.pos[0] - self.Beacons[0]['x'])**2+(self.pos[1] - self.Beacons[0]['y'])**2+(self.pos[2] - self.Beacons[0]['z'])**2) + self.ruido*randn()
        self.Beacons_dist[1] = math.sqrt((self.pos[0] - self.Beacons[1]['x'])**2+(self.pos[1] - self.Beacons[1]['y'])**2+(self.pos[2] - self.Beacons[1]['z'])**2) + self.ruido*randn()
        self.Beacons_dist[2] = math.sqrt((self.pos[0] - self.Beacons[2]['x'])**2+(self.pos[1] - self.Beacons[2]['y'])**2+(self.pos[2] - self.Beacons[2]['z'])**2) + self.ruido*randn()
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

    def get_pos(self):
        return self.pos

    def get_vel(self):
        return self.velcal

    def set_trayec(self,trayec):
        self.trayec = trayec
        self.def_trayec = True
        


if __name__ == "__main__":
    print("main")
    uno =Beacon_sim( dt=0.2, pos=[5,2,1], vel=[2,2,3])
    print(uno.update_position())
    print(uno.get_pos())
    print(uno.update_position())
    print(uno.get_pos())
    print(uno.update_position())
    print(uno.get_pos())

    trayec_x =[]
    trayec_y =[]
    trayec_z =[]
    init_pos = [0.1,0.2,0.3]

    for i in range(0,50,1):
        trayec_x.append(0.0)
        trayec_y.append(0.0)
        trayec_z.append(i*0.1)
        
    for i in range(0,50,1):
        trayec_x.append(trayec_x[-1])
        trayec_y.append(trayec_y[-1]+0.1)
        trayec_z.append(trayec_z[-1])
        
    for i in range(0,50,1):
        trayec_x.append(trayec_x[-1]+0.1)
        trayec_y.append(trayec_y[-1])
        trayec_z.append(trayec_z[-1])
        
    for i in range(0,50,1):
        trayec_x.append(trayec_x[-1])
        trayec_y.append(trayec_y[-1]-0.1)
        trayec_z.append(trayec_z[-1])
    for i in range(0,50,1):
        trayec_x.append(trayec_x[-1]-0.1)
        trayec_y.append(trayec_y[-1])
        trayec_z.append(trayec_z[-1])

    trayec = []
    trayec.append(trayec_x)
    trayec.append(trayec_y)
    trayec.append(trayec_z)
    dos =Beacon_sim( dt=0.2, pos=init_pos, vel=[0,0,0.1])
    dos.set_trayec(trayec)
    for i in range(len(trayec[0]) -1):
        print(i)
        print(dos.update_position())
        print(dos.get_pos())
