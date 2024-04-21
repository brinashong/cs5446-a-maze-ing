import gym
import rospy
import roslaunch
import time
import numpy as np
import math

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty as odomEmpty

from tf.transformations import euler_from_quaternion

from gym.utils import seeding

# initial position of the robot
INIT_X = 0.0
INIT_Y = 0.0
INIT_YAW = 0.0

# goal position
GOAL_X = 8.0
GOAL_Y = 8.0

class GazeboSimpleMazeTurtlebotLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboSimpleMazeTurtlebotLidar_v0.launch")
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_odom_pub = rospy.Publisher("mobile_base/commands/reset_odometry", odomEmpty, queue_size=10)

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.x = INIT_X
        self.y = INIT_Y
        self.yaw = INIT_YAW

        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.yaw = yaw

    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 0.2
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf'):
                    discretized_ranges.append(6)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data.ranges[i]))
            if (min_range > data.ranges[i] > 0):
                done = True
        
        self.front = discretized_ranges[2]
        self.left = min(discretized_ranges[3:4])
        self.right = min(discretized_ranges[0:1])
        # print("f:")
        # print(self.front,self.left,self.right)
        return discretized_ranges, done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def calculate_reward(self, state, action, done):
        # Constants for reward calculation
        GOAL_REWARD = 100
        TIME_PENALTY = -5
        COLLISION_PENALTY = -20
        PROXIMITY_REWARD = 50
        YAW_PENALTY = -10
        WALL_PENALTY = -10
        # Calculate the distance to the goal
        distance_to_goal = self.euclidean_distance(self.x, self.y, GOAL_X, GOAL_Y)
        # Calculate yaw difference to goal
        target_yaw = math.atan2((GOAL_Y - self.y), (GOAL_X - self.x))
        yaw_diff = target_yaw - self.yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        yaw_diff = abs(yaw_diff)
        # Calculate reward based on the state
        if not done:
            if distance_to_goal < 1.0:  # Check if near the goal
                reward = GOAL_REWARD
                done = True
            else:
                # Reward for being closer to the goal, normalized by the initial distance
                reward = (1 - distance_to_goal / self.euclidean_distance(INIT_X, INIT_Y, GOAL_X, GOAL_Y)) * PROXIMITY_REWARD
                # Penalize for tim we
                reward += TIME_PENALTY
                reward += YAW_PENALTY * yaw_diff

                # stay away from walls
                ideal = max(self.left, self.right, self.front)
                if action == 0 and ideal != self.front:
                    reward += WALL_PENALTY
                elif action == 1 and ideal != self.left:
                    reward += WALL_PENALTY
                elif action == 2 and ideal != self.right:
                    reward += WALL_PENALTY
        else:
            # Penalize the robot for hitting an obstacle
            reward = COLLISION_PENALTY

        return reward, done

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.25
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -0.3
            self.vel_pub.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done = self.discretize_observation(data,5)
        info = {}
        if (done == True):
            print("collided at ", self.x, " ", self.y)
            info = "collision"

        reward, done = self.calculate_reward(state, action, done)
        if (info != "collision" and done == True):
            info = "success"

        return state, reward, done, info

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_world service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)

        self.x = INIT_X
        self.y = INIT_Y
        self.reset_odom_pub.publish(odomEmpty())

        # time.sleep(1)

        return state
