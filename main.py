from Environment import  Environment
from rtdp_agent import Agent
import pickle
from copy import deepcopy
import sys

num_cars = 8
num_actions = 21+9 #21 for acceleration, 9 for angular velocity
#pos = [[60,17],[100,17],[120,19],[180,21],[120,15]]
#vel = [0,0.5,0.5,-0.5,0.5]
pos = [[60,17],[100,17],[100,19],[120,21],[120,15], [140, 21], [140, 19], [180, 15]]
vel = [0,0.5,0.5,-0.5,-0.5, -0.5, 0.5, -0.5]

acc = [0,0,0,0,0,0,0,0]
acc_noise = int(sys.argv[1])
angular_vel_z_noise = int(sys.argv[1])
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
avg_dist = 0
final_seq = []
#min_steps = 1000
max_reward = -float("inf")
stats = []
while num_episodes < 1001:
    steps = 0
    reward = -1
    env.reset(num_cars, num_actions, pos, vel, acc, acc_noise,angular_vel_z_noise,
                  acc_resolution, ang_resolution,acc_min, angular_vel_z_min, car_ori)
    env2.reset(num_cars, num_actions, pos, vel, acc, acc_noise, angular_vel_z_noise,
              acc_resolution, ang_resolution, acc_min, angular_vel_z_min, car_ori)
    init_state = tuple([env.pos, env.vel, env.car_ori])
    state = deepcopy(init_state)
    cum_reward = 0
    goal1_reached = 0
    goal2_reached = 0
    state_sequence = [deepcopy(init_state)]
    while not( state[0][0][0] >= 200 or reward == -200) and steps < 1000:
        action = agent.getAction(state, env)
        #print("Taking Action" , str(action))
        state, reward = env.goToNextState(action)
        state_sequence.append(deepcopy(state))
        cum_reward += reward
        if state[0][0][0] >= 200  :
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
    avg_dist += state[0][0][0]
    if num_episodes % 10 == 0 :
        print (num_episodes, steps, reward,cum_reward, env.goals_reached, state)
    if num_episodes % 50 == 0 and num_episodes != 0:
        print (count_success, max_reward, float(avg_reward)/50, float(avg_dist)/50,goal1, goal2)
        stats.append((count_success, max_reward, float(avg_reward)/50, goal1, goal2, float(avg_dist)/50, num_episodes))
        avg_reward = 0
        avg_dist = 0
        count_success = 0
        goal1 = 0
        goal2 = 0
    # if num_episodes > 0 :
    #     if steps < min_steps and state[0][0][0] > 200:
    #         min_steps = steps
    #         final_seq = deepcopy(state_sequence)
    # if num_episodes % 500 == 0 :
    #     pickle.dump(final_seq, open("sequence_noise_1_vel_half_env4" + str(num_episodes) + ".pkl", "wb"))
    #     print len(final_seq)
    #     min_steps = 1000
    if num_episodes > 0 :
        if max_reward < cum_reward and state[0][0][0] > 200:
            max_reward = cum_reward
            final_seq = deepcopy(state_sequence)
    if num_episodes % 500 == 0 :
        pickle.dump(final_seq, open(sys.argv[1] + "/" +"sequence_noise_"+ sys.argv[1] +"_vel_half_env5" + str(num_episodes) + "test.pkl", "wb"))
        print len(final_seq), max_reward
        max_reward = -float("inf")

    # if reward == 1000 :
    #     agent.updateEndEpisode(env2)
    num_episodes +=1

print len(final_seq)
print final_seq
with open(sys.argv[1] + "/" +"sequence_noise_"+ sys.argv[1]+"_vel_half_env5.testpkl", "wb") as f:
    pickle.dump(final_seq, f)

with open(sys.argv[1] + "/" +"sequence_noise_"+ sys.argv[1]+"_vel_half_env5_stats.testpkl", "wb") as f:
    pickle.dump(stats, f)