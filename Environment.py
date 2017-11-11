import numpy as np

class Environment :
    DT = 0.1
    def __init__(self, num_cars,num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise, acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori ):
        self.num_cars = num_cars
        self.pos = []
        self.vel = []
        self.acc = []
        for i in range(0,num_cars) :
            self.pos.append(pos[i])   #(x,y)
            self.vel.append(vel[i])   #(vel_x)
            self.acc.append(acc[i])   #(acc_x)
        self.num_actions = num_actions
        self.acc_noise = acc_noise
        self.angular_vel_z_noise = angular_vel_z_noise
        self.acc_resolution = acc_resolution
        self.ang_resolution = ang_resolution
        self.angular_vel_z_min = angular_vel_z_min
        self.acc_min = acc_min
        self.car_ori = car_ori

    def goToNextState(self, action):
        acc_x = 0
        angular_vel_z = 0
        if action < 0 or action > self.num_actions :
            print "invalid action"
            return
        if action < 21 :
            acc_x = (action-0)*self.acc_resolution + self.acc_min
            acc_x += np.random.normal(0, self.acc_noise)
        else :
            angular_vel_z = (action - 21)*self.ang_resolution + self.angular_vel_z_min
            angular_vel_z += np.random.normal(0, self.angular_vel_z_noise)

        self.pos[0], self.vel[0], self.car_ori = self.getNextPosition(0,acc_x,angular_vel_z )
        for i in range(1,self.num_cars) :
            self.pos[i], self.vel[i],_ = self.getNextPosition(i,self.acc[i],0 )


    def getNextPosition(self, index, acc_x, angular_vel_z):
        global DT
        car_ori = 0
        if index == 0 and angular_vel_z != 0:
            car_ori = self.car_ori + angular_vel_z*DT
        elif angular_vel_z == 0 and index == 0:
            car_ori = self.car_ori
        else :
            car_ori = 0

        vel = self.vel[index] + acc_x*DT
        pos_x = self.pos[index][0] + vel*DT(np.cos(np.deg2rad(car_ori)))
        pos_y = self.pos[index][1] + vel * DT(np.sin(np.deg2rad(car_ori)))
        return [pos_x,pos_y], vel, car_ori


