import numpy as np

from geometry_msgs.msg import PoseStamped


class Object:
    """Object base class"""
    def __init__(self, init_pos, init_ori, frame='world', name='object'):
        """Initialize an Object in the environment

        Arguments
        ----------
        - init_pos: np.ndarray or list or tuple
            Initial position

        - init_ori: np.ndarray or list or tuple
            Initial orientation, quaternion

        - frame: str (default = 'world')
            Reference frame of the object pose

        - name: str (default = 'object')
            Name of the object in MoveIt! planning scene

        Returns
        ----------

        """
        if not isinstance(init_pos, (np.ndarray, list, tuple)) or not isinstance(init_ori, (np.ndarray, list, tuple)):
            raise TypeError("Initial position and orientation must have one of the following types: np.ndarray, "
                            "list or tuple!")

        if len(init_pos) != 3:
            raise ValueError("Initial position must be 3 dimensional array!")

        if len(init_ori) != 4:
            raise ValueError("Initial orientation must be 4 dimensional array!")

        self.init_pos = init_pos
        self.init_ori = init_ori
        self.frame = frame
        self.name = name

        self._init_pose = PoseStamped()
        self._init_pose.header.frame_id = frame
        self._init_pose.pose.position.x = init_pos[0]
        self._init_pose.pose.position.y = init_pos[1]
        self._init_pose.pose.position.z = init_pos[2]
        self._init_pose.pose.orientation.x = init_ori[0]
        self._init_pose.pose.orientation.y = init_ori[1]
        self._init_pose.pose.orientation.z = init_ori[2]
        self._init_pose.pose.orientation.w = init_ori[3]

    @property
    def init_pose(self):
        return self._init_pose
