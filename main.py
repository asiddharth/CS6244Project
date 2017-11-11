from Environment import  Environment
num_cars = 5
num_actions = 21+9 #21 for acceleration, 9 for angular velocity
pos = [[0,0],[0,0],[0,0],[0,0],[0,0]]
vel = [0,0,0,0,0]
acc = [0,0,0,0,0]
acc_noise = 0
angular_vel_z_noise = 0
acc_resolution = 1
ang_resolution = 5
acc_min = -10
angular_vel_z_min = -20
car_ori = 0
env = Environment(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
print (env.vel)