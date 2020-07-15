"""

Reacher task for the UR5 robot

Adapt from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/envs/sawyer/pick_and_place_env.py

"""
import numpy as np
import moveit_commander

from src.envs.ur5 import UR5Env
from src.worlds import BlockWorld
from src.core.serializable import Serializable


class PickAndPlaceEnv(UR5Env, Serializable):
    """Pick and place environment"""
    def __init__(self, initial_goal, initial_joint_pos, sparse_reward=True, simulated=True, distance_threshold=0.05,
                 target_range=0.15, robot_group_name='manipulator', robot_control_mode='position',
                 block_state_topic_name='/mujoco/model_states'):
        """Initialize a PickAndPlaceEnv object

        Arguments
        ----------
        - initial_goal: np.ndarray
            The initial goal for the task

        - initial_joint_pos: {'joint_name': joint_pos}
            Initial joint position

        - sparse_reward: bool (default = True)
            Use sparse reward on True

        - simulated: bool (default = True)
            Run simulated experiment on True

        - distance_threshold: float (default = 0.05)
            Threshold for whether experiment is done

        - target_range: float (default = 0.15)
            Delta range the goal is randomized

        - robot_group_name: str (default = 'manipulator')
            Control group name of the robot

        - robot_control_mode: str (default = 'position')
            Robot control mode, only position control is available now

        - block_state_topic_name: str (default = '/mujoco/model_states')
            Topic name to get the block state for simulated and real robot

        Returns
        ----------

        """
        Serializable.quick_init(self, locals())
        super(PickAndPlaceEnv, self).__init__(initial_goal, initial_joint_pos, sparse_reward, distance_threshold,
                                              robot_group_name, robot_control_mode)
        self.target_range = target_range
        self.simulated = simulated
        self.block_state_topic_name = block_state_topic_name

        self._moveit_scene = moveit_commander.PlanningSceneInterface()
        self._world = BlockWorld(self._moveit_scene, self._robot.moveit_robot.get_planning_frame(),
                                 simulated=self.simulated, resource=self.block_state_topic_name)

    def _sample_goal(self):
        """Sample a random goal position

        Arguments
        ----------

        Returns
        ----------
        - goal: np.ndarray
            The new sampled goal

        """
        goal = self.initial_goal.copy()

        random_goal_delta = np.random.uniform(
            -self.target_range, self.target_range, size=2)
        goal[:2] += random_goal_delta

        return goal

    def reset(self):
        """Resets the state of the environment, returning an initial observation

        Arguments
        ----------

        Returns
        ----------
        - intial_observation: np.ndarray
            The initial observation of the task

        """
        initial_observation = super(PickAndPlaceEnv, self).reset()
        self._world.reset()

        return initial_observation

    def close(self):
        """Clean up the environment

        Arguments
        ----------

        Returns
        ----------

        """
        self._world.close()
        return super(ReacherEnv, self).close()
