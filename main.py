from Environment import  Environment
from rtdp_agent import Agent
num_cars = 5
num_actions = 21+9 #21 for acceleration, 9 for angular velocity
pos = [[100,17],[140,17],[180,23],[10,17],[10,17]]
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
env2 = Environment(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
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
count_success = 0
while num_episodes < 5001 :
    steps = 0
    reward = -1
    env.reset(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
    env2.reset(num_cars, num_actions, pos, vel, acc, acc_noise, angular_vel_z_noise,
              acc_resolution, ang_resolution, acc_min, angular_vel_z_min, car_ori)
    init_state = tuple([env.pos, env.vel, env.car_ori])
    state = init_state
    cum_reward = 0
    while not(reward == 1000 or reward == -200) and steps < 1000:
        action = agent.getAction(state, env)
        #print("Taking Action" , str(action))
        state, reward = env.goToNextState(action)
        cum_reward += reward
        if reward == 1000 :
            count_success +=1
        #print("Next State, Reward" ,state, str(reward))
        steps +=1
        #print steps
    if num_episodes % 10 == 0 :
        print (num_episodes, steps, reward,cum_reward, state)
    if num_episodes % 500 == 0 :
        print (count_success)
        count_success = 0
    if reward == 1000 :
        agent.updateEndEpisode(env2)
    num_episodes +=1

