from Environment import  Environment
from rtdp_agent import Agent
num_cars = 5
num_actions = 21+9 #21 for acceleration, 9 for angular velocity
pos = [[60,17],[100,17],[120,19],[140,21],[180,17]]
vel = [0,0.5,0.5,0.5,0.5]
acc = [0,0,0,0,0]
acc_noise = 0.1
angular_vel_z_noise = 0.1
acc_resolution = 0.5
ang_resolution = 10
acc_min = -5
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
avg_reward = 0
goal1 = 0
goal2 = 0
while num_episodes < 5001:
    steps = 0
    reward = -1
    env.reset(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
    env2.reset(num_cars, num_actions, pos, vel, acc, acc_noise, angular_vel_z_noise,
              acc_resolution, ang_resolution, acc_min, angular_vel_z_min, car_ori)
    init_state = tuple([env.pos, env.vel, env.car_ori])
    state = init_state
    cum_reward = 0
    goal1_reached = 0
    goal2_reached = 0
    while not(reward == 1200 or reward == -200) and steps < 1000:
        action = agent.getAction(state, env)
        #print("Taking Action" , str(action))
        state, reward = env.goToNextState(action)
        cum_reward += reward
        if reward == 1200 :
            count_success +=1
        if env.goals_reached ==1 and goal1_reached == 0:
            goal1_reached = 1
            goal1 +=1
        if env.goals_reached == 2 and goal2_reached == 0:
            goal2_reached = 1
            goal2 +=1
        #print("Next State, Reward" ,state, str(reward), env.goals_reached)
        steps +=1
    #print steps
    avg_reward += cum_reward
    if num_episodes % 10 == 0 :
        print (num_episodes, steps, reward,cum_reward, env.goals_reached, state)
    if num_episodes % 500 == 0 :
        print (count_success, float(avg_reward)/500, goal1+goal2, goal2)
        avg_reward = 0
        count_success = 0
        goal1 = 0
        goal2 = 0
    # if reward == 1000 :
    #     agent.updateEndEpisode(env2)
    num_episodes +=1

