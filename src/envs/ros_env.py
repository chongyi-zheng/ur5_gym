"""

ROS gym environment

Adapt from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/envs/ros_env.py

"""


import gym
# import numpy as np

from src.core.serializable import Serializable


class RosEnv(gym.Env, Serializable):
    """Superclass for all ros environment"""
    def __init__(self):
        """Initialize a RosEnv object

        Arguments
        ----------

        Returns
        ----------

        """
        super(RosEnv, self).__init__()
        Serializable.quick_init(self, locals())

        self._initial_setup()

    def initialize(self):
        # TODO (gh/74: Add initialize interface for robot)
        pass

    # =======================================================
    # The functions that base sawyer.garage.Env asks to implement
    # =======================================================
    def step(self, action):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    @property
    def action_space(self):
        raise NotImplementedError

    @property
    def observation_space(self):
        raise NotImplementedError

    def _initial_setup(self):
        raise NotImplementedError

    # ====================================================
    # Need to be implemented in specific robot env
    # ====================================================
    def sample_goal(self):
        """
        Samples a new goal and returns it.
        """
        raise NotImplementedError

    def get_observation(self):
        """
        Get observation
        """
        raise NotImplementedError

    def done(self, achieved_goal, goal):
        """
        :return if done: bool
        """
        raise NotImplementedError

    def reward(self, achieved_goal, goal):
        """
        Compute the reward for current step.
        """
        raise NotImplementedError

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, value):
        self._goal = value
