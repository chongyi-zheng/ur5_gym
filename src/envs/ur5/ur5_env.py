# from src.envs.ros_env import RosEnv

import gym
import abc
import collections

from src.core.serializable import Serializable
from src.robots.ur5 import UR5
from src.util.common import rate_limited, goal_distance

STEP_FREQ = 20


class UR5Env(gym.Env, Serializable, metaclass=abc.ABCMeta):
    def __init__(self, initial_goal, initial_joint_pos, sparse_reward=True, distance_threshold=0.05,
                 robot_group_name='manipulator', robot_control_mode='position'):
        """Initialize a UR5Env object

        Arguments
        ----------
        - initial_goal: np.ndarray
            The initial goal for the task

        - initial_joint_pos: {'joint_name': joint_pos}
            Initial joint position

        - sparse_reward: bool (default = True)
            Use sparse reward on True

        - distance_threshold: float (default = 0.05)
            Threshold for whether experiment is done

        - robot_group_name: str (default = 'manipulator')
            Control group name of the robot

        - robot_control_mode: str (default = 'position')
            Robot control mode, only position control is available now

        Returns
        ----------

        """
        super(UR5Env, self).__init__()
        Serializable.quick_init(self, locals())

        self.initial_goal = initial_goal
        self.initial_joint_pos = initial_joint_pos
        self.sparse_reward = sparse_reward
        self.distance_threshold = distance_threshold
        self.robot_group_name = robot_group_name
        self.robot_control_mode = robot_control_mode

        self._goal = None
        self._robot = UR5(initial_joint_pos, group_name=robot_group_name, control_mode=robot_control_mode)

        self._setup_spaces()

    def _setup_spaces(self):
        """Setup observation and action spaces

        Arguments
        ----------

        Returns
        ----------

        """
        self.observation_space = self._robot.observation_space
        self.action_space = self._robot.action_space

    # def _initial_setup(self):
    #     self._robot.reset()
    #
    # def shutdown(self):
    #     self._world.terminate()

    @abc.abstractmethod
    def _sample_goal(self):
        """Sample goal

        Arguments
        ----------

        Returns
        ----------

        """
        raise NotImplementedError

    def _get_observation(self):
        """Get current observation

        Arguments
        ----------

        Returns
        ----------
        - observation: {'observation': obs, 'achieved_goal': achieved_goal, 'desired_goal': self.goal}

        """
        obs = self._robot.get_observation()
        achieved_goal, _ = self._robot.get_ee_pose()

        Observation = collections.namedtuple(
            'Observation', 'observation achieved_goal desired_goal')

        observation = Observation(
            observation=obs,
            achieved_goal=achieved_goal,
            desired_goal=self._goal)

        return observation

    def compute_reward(self, achieved_goal, goal):
        """Compute the reward for current step

        Arguments
        ----------
        - achieved_goal: np.ndarray
            Current achieved goal

        - goal: np.ndarray
            Desired goal

        Returns
        ----------
        - reward: float
            Reward for the current step

        """
        d = goal_distance(achieved_goal, goal)
        if d < self.distance_threshold:
            reward = 0.0  # 100
        else:
            if self.sparse_reward:
                reward = -1.0
            else:
                reward = -d

        return reward

    def reset(self):
        """Resets the state of the environment, returning an initial observation

        Arguments
        ----------

        Returns
        ----------
        - intial_observation: np.ndarray
            The initial observation of the task

        """
        self._robot.reset()
        self._goal = self._sample_goal()
        initial_observation = self._get_observation().observation
        return initial_observation

    @rate_limited(STEP_FREQ)
    def step(self, action):
        """Perform a step in simulator or on real robot. When end of episode is reached, reset() should be called to
        reset the environment's internal state.

        Arguments
        ----------
        - action: np.ndarray
            an action provided by the environment

        Returns
        ----------
        - observation: np.ndarray
            Agent's observation of the current environment

        - reward: float
            Amount of reward due to the previous action

        - done: bool
            A boolean, indicating whether the episode has ended
        - info: {'info_name': info_value}
            A dictionary containing other diagnostic information from the previous action

        """
        self._robot.send_command(action)

        obs = self._get_observation()

        observation = obs.observation
        reward = self.compute_reward(obs.achieved_goal, self._goal)
        done = not self._robot.safety_check()  # done = True iff robot is in collision state
        info = {
            'is_success': goal_distance(obs.achieved_goal, self._goal) <= self.distance_threshold
        }

        return observation, reward, done, info

    # @property
    # def action_space(self):
    #     return self._robot.action_space

    def render(self, mode='human'):
        pass  # no implementation now
        # return super(UR5Env, self).render(mode)
