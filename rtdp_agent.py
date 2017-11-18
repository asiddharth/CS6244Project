from Environment import  Environment
import  numpy as np
num_samples = 1
gamma = 1
class Agent:
    def __init__(self) :
        self.value_dict = {}
        self.need_update = []

    def getValue(self, state, env) :
        value = self.value_dict.get(state, self.getHeuristic(state, env))
        return value

    def updateValue(self, state, value) :
        self.value_dict[state] = value

    def getAction(self, state, env) :
        Q = [0 for i in range(env.num_actions)]
        #self.need_update.append(state)
        for i in range(env.num_actions) :
            cum_reward = 0
            for j in range(num_samples) :
                next_state, reward = env.checkNextState(i)
                cum_reward += gamma*self.getValue(self.parseState(next_state,env.num_cars ), env) + reward # TODO : NEXT STATE MIGHT NOT
            Q[i] = round(float(cum_reward)/num_samples,5)
        maxQ = np.max(np.array(Q))
        listQ = []
        for ind in range(len(Q)) :
            if  Q[ind] == maxQ :
                listQ.append(ind)
        action = np.random.randint(0, len(listQ), 1)
        self.updateValue(self.parseState(state,env.num_cars ), Q[listQ[action[0]]])
        return listQ[action[0]]

    def updateEndEpisode(self, env):
        for indddddd in range(5) :
            for state in reversed(self.need_update) :
                env.setState(state)
                Q = [0 for i in range(env.num_actions)]
                for i in range(env.num_actions):
                    cum_reward = 0
                    for j in range(max(num_samples/5, 1)):
                        next_state, reward = env.checkNextState(i)
                        cum_reward += gamma * self.getValue(
                            self.parseState(next_state, env.num_cars), env) + reward  # TODO : NEXT STATE MIGHT NOT
                    Q[i] = round(float(cum_reward) / num_samples, 5)
                maxQ = np.max(np.array(Q))
                listQ = []
                for ind in range(len(Q)):
                    if Q[ind] == maxQ:
                        listQ.append(ind)
                action = np.random.randint(0, len(listQ), 1)
                if (indddddd == 4 ) :
                    print(Q[listQ[action[0]]],state )
                self.updateValue(self.parseState(state, env.num_cars), Q[listQ[action[0]]])
        self.need_update = []

    def parseState(self, state, num_cars):
        pos = state[0]
        vel = state[1]
        car_ori = state[2]
        parsedState = [0 for i in range( 4 +2 * 4 + 1)]
        [car_x, car_y] = pos[0]
        parsedState[0],parsedState[1] = car_x,car_y
        parsedState[2], parsedState[3] = vel[0], car_ori
        min1 = float("inf")
        min2 = float("inf")
        min3 = float("inf")
        min4 = float("inf")
        min_dist = [-1,-1,-1,-1]
        tile_size = 7
        dist_max = 20
        for i in range(1,num_cars) :
            dist = (pos[i][0]-car_x)**2 + (pos[i][1]-car_y)**2
            if dist < min1 :
                min1 = dist
                min_dist = [i, min_dist[0], min_dist[1], min_dist[3]]
            elif dist < min2 :
                min2 = dist
                min_dist = [min_dist[0], i,min_dist[1], min_dist[3]]
            elif dist < min3 :
                min3 = dist
                min_dist = [min_dist[0], min_dist[1],i, min_dist[3]]
            elif dist < min4 :
                min4 = dist
                min_dist = [min_dist[0], min_dist[1], min_dist[3],i]
        for i in range(4) :
            if np.sum((np.array(pos[min_dist[i]]) - np.array(pos[0])) ** 2) < dist_max :
                [parsedState[4+2*i], parsedState[4+2*i +1]] = list(np.array(pos[min_dist[i]]) - np.array(pos[0]))
            else :
                [parsedState[4 + 2 * i], parsedState[4 + 2 * i + 1]] = [-1,-1]

        for i in min_dist :
            if pos[i][0] > car_x :
                parsedState[len(parsedState) - 1] = (pos[i][0]-car_x)**2 + (pos[i][1]-car_y)**2
                break
        if parsedState[len(parsedState) - 1] == 0:
            min = float("inf")
            for i in range(1,num_cars) :
                if pos[i][0] > car_x:
                    dist = (pos[i][0]-car_x)**2 + (pos[i][1]-car_y)**2
                    if dist< min :
                        parsedState[len(parsedState) - 1] = dist
                        min = dist
        if parsedState[len(parsedState) - 1] == 0:
            parsedState[len(parsedState) - 1] = -1

        return tuple([round(round(i,0)/tile_size,0) for i in parsedState])

    def getHeuristic(self, state, env):
        if env.checkCollision([state[0],state[1]], state[3]) :
            return -500
        if state[0] >= 200 :
            return 0
        return round(float(state[0] - 200)/0.5, 3)
