from Environment import  Environment
import  numpy as np
num_samples = 10
gamma = 1
class Agent:
    value_dict = {}
    def getValue(self, state) :
        value = self.value_dict.get(state, self.getHeuristic(state))
        return value

    def updateValue(self, state, value) :
        self.value_dict[state] = value

    def getAction(self, state, env) :
        Q = [0 for i in range(env.num_actions)]
        for i in range(env.num_actions) :
            cum_reward = 0
            for i in range(num_samples) :
                next_state, reward = env.checkNextState(i)
                cum_reward += gamma*self.getValue(self.parseState(next_state,env.num_cars )) + reward
            Q[i] = float(cum_reward)/num_samples
        action = np.argmax(np.array(Q))
        self.updateValue(self.parseState(state,env.num_cars ), Q[action])
        return action

    def parseState(self, state, num_cars):
        pos = [[] for i in num_cars]
        vel = [0 for i in num_cars]
        car_ori = 0

        for i in range(num_cars) :
            pos[i][0] = state[2*i]
            pos[i][1] = state[2 * i + 1]
        for i in range(num_cars) :
            vel[i][0] = state[2*i + 2*num_cars]
            vel[i][1] = state[2 * i + 1 + 2*num_cars]
        car_ori = state(len(state)-1)

        parsedState = [0 for i in range(2*6 + 4 +1 )]

        return state

    def getHeuristic(self, state):
        return (state[0] - 200)/5
