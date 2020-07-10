"""

World without any objects

Based on https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/worlds/empty_world.py

"""

import collections

from geometry_msgs.msg import PoseStamped
import gym
import numpy as np

from src.worlds.world import World


class EmptyWorld(World):
    """Empty world class"""
    def __init__(self, moveit_scene, frame_id, simulated=False):
        """Users use this to manage world and get world state

        Arguments
        ----------
        - moveit_scene: moveit_commander.PlanningSceneInterface
            Use this to add/Move/Remove objects in MoveIt!

        - frame_id: string
            Use this to add/Move/Remove objects in MoveIt!, reference frame

        - simulated: bool
            If simulated

        Returns
        ----------

        """
        self.moveit_scene = moveit_scene
        self.frame_id = frame_id
        self.simulated = simulated

        self._observation_space = gym.spaces.Box(-np.inf, np.inf, shape=self.get_observation().observation.shape,
                                                 dtype=np.float32)

    @property
    def observation_space(self):
        """Get observation space of the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        return self._observation_space

    def reset(self):
        """Reset the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        # Add table to moveit
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose.position.x = 0.655
        pose_stamped.pose.position.y = 0

        # Leave redundant space
        pose_stamped.pose.position.z = -0.02
        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1.0

        self.moveit_scene.add_box('table', pose_stamped, (1.0, 0.9, 0.1))

    def close(self):
        """Terminate the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        self.moveit_scene.remove_world_object('table')

    def get_observation(self):
        """Get the observation from empty world

        Arguments
        ----------

        Returns
        ----------
        - obs: {observation: }


        """
        achieved_goal = np.array([])
        obs = np.array([])

        Observation = collections.namedtuple('Observation',
                                             'observation achieved_goal')

        observation = Observation(observation=obs, achieved_goal=achieved_goal)

        return observation
