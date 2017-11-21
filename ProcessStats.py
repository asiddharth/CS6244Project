import pickle
import matplotlib.pyplot as plt
name = "env5"
blah = pickle.load(open("sequence_noise_1_vel_half_env5_stats.pkl", "rb"))
print blah
mapping=  {}
for entry in blah :
    mapping[entry[len(entry)-1]] = list(entry[:-1])

count_success =[0 for i in range(len(mapping))]
max_reward = [0 for i in range(len(mapping))]
avg_reward = [0 for i in range(len(mapping))]
goal1 = [0 for i in range(len(mapping))]
goal2 = [0 for i in range(len(mapping))]
avg_dist = [0 for i in range(len(mapping))]

for key in sorted(mapping.iterkeys()):
    print key, int(key/50), len(count_success)
    [count_success[int(key/50)-1], max_reward[int(key/50)-1], avg_reward[int(key/50)-1], goal1[int(key/50)-1],\
    goal2[int(key/50)-1], avg_dist[int(key/50)-1]]= mapping[key]


plt.plot(sorted(mapping.iterkeys()), avg_reward)
plt.xlabel('Runs')
plt.ylabel('Average Reward')
plt.title('Reward v/s Runs')
plt.savefig('reward' + name + '.png')
plt.show()

plt.plot(sorted(mapping.iterkeys()), avg_dist)
plt.xlabel('Runs')
plt.ylabel('Average Distance')
plt.title('Distance v/s Runs')
plt.savefig('distance' + name +'.png')
plt.show()

plt.plot(sorted(mapping.iterkeys()), max_reward)
plt.xlabel('Runs')
plt.ylabel('Max Reward')
plt.title('Max Reward v/s Runs')
plt.savefig('max_reward' + name +'.png')
plt.show()

plt.plot(sorted(mapping.iterkeys()), count_success)
plt.xlabel('Runs')
plt.ylabel('Successes')
plt.title('Successes v/s Runs')
plt.savefig('successes' + name +'.png')
plt.show()

plt.plot(sorted(mapping.iterkeys()), goal1)
plt.xlabel('Runs')
plt.ylabel('Goal1')
plt.title('Goal1 v/s Runs')
plt.savefig('goal1' + name +'.png')
plt.show()


plt.plot(sorted(mapping.iterkeys()), goal2)
plt.xlabel('Runs')
plt.ylabel('Goal2')
plt.title('Goal2 v/s Runs')
plt.savefig('goal2' + name +'.png')
plt.show()