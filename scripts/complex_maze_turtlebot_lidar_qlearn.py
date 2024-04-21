#!/usr/bin/env python
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import qlearn
import liveplot

def render():
    render_skip = 0 #Skip first X episodes.
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':

    env = gym.make('GazeboComplexMazeTurtlebotLidar-v0')

    outdir = '/home/developer/reference/cs5446-a-maze-ing/experiments/qlearn_complex'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.5, gamma=0.8, epsilon=0.9)

    initial_epsilon = qlearn.epsilon

    epsilon_discount = 0.99 #0.9986

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

        # time.sleep(1)
        observation = env.reset()

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        #render() #defined above, not env.render()

        state = ''.join(map(str, observation))

        for i in range(1500):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward

            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            qlearn.learn(state, action, reward, nextState)

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
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))

        total_reward += cumulated_reward

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    avg_steps_to_collision = sum(fail_results) / len(fail_results)
    if (num_success > 0):
        avg_steps_to_success = sum(success_results) / len(success_results)

    # write results to file
    with open('/home/developer/reference/cs5446-a-maze-ing/experiments/qlearn_complex/results.txt', 'w') as file:
        file.write("num_collisions: " + str(num_collisions) + "\n")
        file.write("num_success: " + str(num_success) + "\n")
        file.write("avg steps to collision: " + str(avg_steps_to_collision) + "\n")
        if (num_success > 0):
            file.write("avg steps to success: " + str(avg_steps_to_success) + "\n")
        file.write("avg reward per episode: " + str(total_reward/total_episodes) + "\n")

    env.close()

