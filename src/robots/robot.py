 
"""

Robot Interface

Taken from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/robots/robot.py

"""


class Robot:
    def __init__(self):
        """Initialize a Robot object

        Arguments
        ----------

        Returns
        ----------

        """
        pass

    def reset(self):
        """User uses this to reset the robot at the beginning of every training episode

        Arguments
        ----------

        Returns
        ----------

        """
        raise NotImplementedError

    def send_command(self, commands):
        """User uses this to send commands to robot

        Arguments
        ----------
        - commands: np.ndarray
            The control command

        Returns
        ----------

        """
        raise NotImplementedError

    @property
    def action_space(self):
        """User uses this to get robot's action_space

        Arguments
        ----------

        Returns
        ----------

        """
        raise NotImplementedError

    def get_observation(self):
        """User uses this to get the most recent observation of robot

        Arguments
        ----------

        Returns
        ----------

        """
        raise NotImplementedError

    @property
    def observation_space(self):
        """User uses this to get robot's observation_space

        Arguments
        ----------

        Returns
        ----------

        """
        raise NotImplementedError
