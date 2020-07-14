"""

World without any objects

Based on https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/worlds/empty_world.py

"""

import collections

from geometry_msgs.msg import PoseStamped
import gym
import numpy as np

from src.worlds.world import World
from src.objects import BoxTable


class EmptyWorld(World):
    """Empty world"""
    def __init__(self, moveit_scene, frame_id, simulated=True):
        """Initialize an EmptyWorld object

        Arguments
        ----------
        - moveit_scene: moveit_commander.PlanningSceneInterface
            Use this to add/Move/Remove objects in MoveIt!

        - frame_id: string
            Use this to add/Move/Remove objects in MoveIt!, reference frame

        - simulated: bool (default = True)
            If simulated

        Returns
        ----------

        """
        self.moveit_scene = moveit_scene
        self.frame_id = frame_id
        self.simulated = simulated

        self._box_table = None
        self._observation_space = None

        self._initialize_world()

    def _initialize_world(self):
        """Initialize the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        self._box_table = BoxTable(self.frame_id)
        self._observation_space = gym.spaces.Box(-np.inf, np.inf, shape=self.get_observation().observation.shape,
                                                 dtype=np.float32)

        self.moveit_scene.add_box(self._box_table.name, self._box_table.init_pose,
                                  self._box_table.size)

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
        pass

    def close(self):
        """Terminate the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        self.moveit_scene.remove_world_object(self._box_table.name)

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
