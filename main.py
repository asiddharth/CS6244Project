from Environment import  Environment
from rtdp_agent import Agent
num_cars = 5
num_actions = 21+9 #21 for acceleration, 9 for angular velocity
pos = [[100,0],[120,0],[10,0],[10,0],[10,0]]
vel = [0,0,0,0,0]
acc = [0,0,0,0,0]
acc_noise = 0
angular_vel_z_noise = 0
acc_resolution = 1
ang_resolution = 10
acc_min = -10
angular_vel_z_min = -40
car_ori = 0
env = Environment(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
# env.goToNextState(21)
# print (env.vel, env.pos, env.car_ori)
# print(env.checkNextState(21))
# print (env.vel, env.pos, env.car_ori)
# print (env.getBoundingBox(env.pos[0], env.car_ori))
# print (env.checkCollision(env.pos[0], env.car_ori))
# print (env.getReward(env.pos[0], env.car_ori))


agent = Agent()
num_episodes = 0
while num_episodes < 1000 :
    steps = 0
    reward = -1
    env.reset(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
    init_state = tuple([env.pos, env.vel, env.car_ori])
    state = init_state
    while reward == -1 and steps < 1000:
        action = agent.getAction(state, env)
        #print("Taking Action" , str(action))
        state, reward = env.goToNextState(action)
        #print("Next State, Reward" ,state, str(reward))
        steps +=1
    print (num_episodes, steps, reward, state)
    print len(agent.value_dict)
    num_episodes +=1

