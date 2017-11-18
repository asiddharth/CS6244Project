import numpy as np

DT = 0.1

class Environment :
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
        pos = self.pos
        vel = self.vel
        car_ori = self.car_ori

        return tuple([pos, vel, car_ori]) , self.getReward(self.pos[0], self.car_ori, self.vel[0])

    def checkNextState (self, action) :
        acc_x = 0
        angular_vel_z = 0
        if action < 0 or action > self.num_actions:
            print "invalid action"
            return
        if action < 21:
            acc_x = (action - 0) * self.acc_resolution + self.acc_min
            acc_x += np.random.normal(0, self.acc_noise)
        else:
            angular_vel_z = (action - 21) * self.ang_resolution + self.angular_vel_z_min
            angular_vel_z += np.random.normal(0, self.angular_vel_z_noise)
        pos = [[] for i in range(self.num_cars)]
        vel = [0 for i in range(self.num_cars) ]
        pos[0], vel[0], car_ori = self.getNextPosition(0, acc_x, angular_vel_z)
        for i in range(1, self.num_cars):
            pos[i], vel[i], _ = self.getNextPosition(i, self.acc[i], 0)
        return tuple([pos, vel, car_ori]), self.getReward(pos[0], car_ori,vel[0])


    def getNextPosition(self, index, acc_x, angular_vel_z):
        global DT
        car_ori = 0
        if index == 0 and angular_vel_z != 0:
            car_ori = round(self.car_ori + angular_vel_z*DT,3)
        elif angular_vel_z == 0 and index == 0:
            car_ori = round(self.car_ori,3)
        else :
            car_ori = 0

        vel = round(min(max(self.vel[index] + acc_x*DT, -5),5),3)
        pos_x = round(self.pos[index][0] + vel*DT*(np.cos(np.deg2rad(car_ori))),3)
        pos_y = round(self.pos[index][1] + vel * DT*(np.sin(np.deg2rad(car_ori))),3)
        return [pos_x,pos_y], vel, car_ori

    def getBoundingBox(self, pos, car_ori) :
        e = np.array([[1.98,0.6], [1.98,-0.6], [0.405, 0.6],[-0.405,-0.6] ])
        rotationMatrix = np.array([[np.cos(np.deg2rad(car_ori)), -np.sin(np.deg2rad(car_ori))], [np.sin(np.deg2rad(car_ori)), np.cos(np.deg2rad(car_ori))]])
        vec1 = np.dot(rotationMatrix, e.T)
        max = np.max(vec1,axis=1)
        min = np.min(vec1, axis=1)
        return round(pos[0] + min[0],3), round(pos[1] + min[1],3), round(pos[0] + max[0],3), round(pos[1] + max[1],3)

    def checkCollision(self, pos, ori):
        gap = 0.5
        min_x, min_y, max_x, max_y = self.getBoundingBox(pos, ori)

        mid_pos = [float(min_x + max_x)/2 , float(min_y + max_y)/2]
        x_expand = float(max_x - min_x) / 2
        y_expand = float(max_y - min_y) / 2
        expanded_pos =[0,0,0,0]
        collision = False
        for i in range(1, self.num_cars) :
            expanded_pos[0] = self.pos[i][0] - 0.405 - x_expand - gap
            expanded_pos[1] = self.pos[i][0] + 1.98 + x_expand + gap
            expanded_pos[2] = self.pos[i][1] -0.6 - y_expand - gap
            expanded_pos[3] = self.pos[i][1] + 0.6 + y_expand + gap

            if mid_pos[0] > expanded_pos[0] and mid_pos[0] < expanded_pos[1] and mid_pos[1] > expanded_pos[2] and mid_pos[1] < expanded_pos[3] :
                collision = True
                break

        return collision

    def getReward(self, pos, car_ori,vel):
        reward = -5
        collision = self.checkCollision(pos, car_ori)
        if collision or pos[1] < 14 or pos[1] > 26:
            reward = -200 + pos[0] - 100
        elif pos[0] >= 200 :
            reward = 1000
        else :
            if pos[1] < 16.4 or pos[1] > 23.6 :
                reward = -(np.exp(0.2*np.min([np.abs(pos[1] - 16.4) ,np.abs(pos[1] - 23.6)  ]))) -1 -1 + (vel)*DT*(np.cos(np.deg2rad(car_ori)))
            else :
                reward =  (-(np.min([(np.abs(pos[1] - 16.4)), (np.abs(pos[1] - 23.6))]))) -1 + (vel)*DT*(np.cos(np.deg2rad(car_ori)))
            if np.abs(vel) <=  0.5 :
                reward -= 5
        return reward

    def reset(self, num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori):
        self.__init__(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)

    def setState(self, state):
        self.pos = state[0]
        self.vel = state[1]
        self.car_ori = state[2]




