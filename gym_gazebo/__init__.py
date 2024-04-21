import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Turtlebot envs
############ START - added by brina #############
register(
    id='GazeboSimpleMazeTurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboSimpleMazeTurtlebotLidarEnv',
    # More arguments here
)
register(
    id='GazeboComplexMazeTurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboComplexMazeTurtlebotLidarEnv',
    # More arguments here
)
############ END - added by brina #############
