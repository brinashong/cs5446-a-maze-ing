#!/usr/bin/env python
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import liveplot
import sarsa


if __name__ == '__main__':

    env = gym.make('GazeboSimpleMazeTurtlebotLidar-v0')

    outdir = '/home/developer/reference/cs5446-a-maze-ing/experiments/sarsa'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    sarsa = sarsa.Sarsa(actions=range(env.action_space.n),
                    epsilon=0.9, alpha=0.5, gamma=0.8)

    initial_epsilon = sarsa.epsilon

    epsilon_discount = 0.95 #0.9986

    start_time = time.time()
    total_episodes = 1000 #10000
    highest_reward = 0
    num_collisions = 0
    num_success = 0
    fail_results = []
    success_results = []
    total_reward = 0

    for x in range(total_episodes):
        done = False

        cumulated_reward = 0 #Should going forward give more reward then L/R ?

        observation = env.reset()

        if sarsa.epsilon > 0.05:
            sarsa.epsilon *= epsilon_discount

        #render() #defined above, not env.render()

        state = ''.join(map(str, observation))

        for i in range(1500):

            # Pick an action based on the current state
            action = sarsa.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward

            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))
            nextAction = sarsa.chooseAction(nextState)

            #sarsa.learn(state, action, reward, nextState)
            sarsa.learn(state, action, reward, nextState, nextAction)

            env._flush(force=True)

            if not(done):
                state = nextState
            else:
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                if (info == "collision"):
                    num_collisions += 1
                    fail_results.append(int(i + 1))
                elif (info == "success"):
                    num_success += 1
                    success_results.append(int(i + 1))
                break

        if not(done):
            env.stats_recorder.save_complete()
            env.stats_recorder.done = True

        if x%100==0:
            plotter.plot(env)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(sarsa.alpha,2))+" - gamma: "+str(round(sarsa.gamma,2))+" - epsilon: "+str(round(sarsa.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))

        total_reward += cumulated_reward

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(sarsa.alpha)+"|"+str(sarsa.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    avg_steps_to_collision = sum(fail_results) / len(fail_results)
    if (num_success > 0):
        avg_steps_to_success = sum(success_results) / len(success_results)

    # write results to file
    with open('/home/developer/reference/cs5446-a-maze-ing/experiments/sarsa/results.txt', 'w') as file:
        file.write("num_collisions: " + str(num_collisions) + "\n")
        file.write("num_success: " + str(num_success) + "\n")
        file.write("avg steps to collision: " + str(avg_steps_to_collision) + "\n")
        if (num_success > 0):
            file.write("avg steps to success: " + str(avg_steps_to_success) + "\n")
        file.write("avg reward per episode: " + str(total_reward/total_episodes) + "\n")

    env.close()
