from src.envs.ros_env import RosEnv
from src.util.common import rate_limited

STEP_FREQ = 50


class SawyerEnv(RosEnv):
    def __init__(self):
        super(RosEnv, self).__init__(self)

    def _initial_setup(self):
        self._robot.reset()
        self._world.initialize()

    def shutdown(self):
        self._world.terminate()

    def reset(self):
        """
        Resets the state of the environment, returning an initial observation.
        Outputs
        -------
        observation :
            the initial observation of the space.
            (Initial reward is assumed to be 0.)
        """
        self._robot.reset()
        self._world.reset()
        self.goal = self.sample_goal()
        initial_observation = self.get_observation().observation
        return initial_observation

    @rate_limited(STEP_FREQ)
    def step(self, action):
        """
        Perform a step in gazebo. When end of episode
        is reached, reset() should be called to reset
        the environment's internal state.
        Input
        -----
        action : an action provided by the environment
        Outputs
        -------
        (observation, reward, done, info)
        observation :
            agent's observation of the current environment
        reward: float
            amount of reward due to the previous action
        done :
            a boolean, indicating whether the episode has ended
        info :
            a dictionary containing other diagnostic information
            from the previous action
        """
        self._robot.send_command(action)

        obs = self.get_observation()

        reward = self.reward(obs.achieved_goal, self.goal)
        done = self.done(obs.achieved_goal, self.goal)
        next_observation = obs.observation
        return Step(observation=next_observation, reward=reward, done=done)

    @property
    def action_space(self):
        return self._robot.action_space
