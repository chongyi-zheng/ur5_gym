"""

UR5 Interface.

Adapt from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/robots/sawyer.py

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gym
import moveit_msgs.msg
import moveit_commander
from sensor_msgs.msg import JointState

import numpy as np
import rospy

from src.robots.kinematics_interface import StateValidity
from src.robots.robot import Robot


POS_DELTA_FACTOR = 0.1


class UR5(object, Robot):
    """UR5 class."""
    def __init__(self, initial_joint_pos, group_name, node_name='ur5_robot', js_topic_name='/joint_states',
                 control_mode='position'):
        """Initialize a UR5 robot

        Arguments
        ----------
        - initial_joint_pos: {str: float}
            {'joint_name': position_value}, and also initial_joint_pos should include all of the 
            joints that user wants to control and observe.
        
        - group_name: str
            Name of the MoveIt! group

        - node_name: str (default = 'ur5_robot')
            Name of the ROS node

        - js_topic_name: str (default = '/joint_states)
            Name of the joint state topic

        - control_mode: str (default = 'position')
            robot control mode, position or velocity or effort
        
        Returns
        ----------
        
        """
        super(UR5, self).__init__()
        # self._limb = intera_interface.Limb('right')
        # self._gripper = intera_interface.Gripper()
        self.initial_joint_pos = initial_joint_pos
        self.group_name = group_name
        self.node_name = node_name
        self.js_topic_name = js_topic_name
        self.control_mode = control_mode

        rospy.init_node(self.node_name, anonymous=True)
        self._moveit_robot = moveit_commander.RobotCommander()
        self._moveit_group = moveit_commander.MoveGroupCommander(self.group_name)
        self._joint_names = self._moveit_group.get_active_joints()
        self._sv = StateValidity()
        self._observation_space = None
        self._action_space = None

        self._setup_spaces()

    def _setup_spaces(self):
        """Setup observation and action spaces

        Arguments
        ----------

        Returns
        ----------

        """
        self._observation_space = gym.spaces.Box(-np.inf, np.inf, shape=self.get_observation().shape, dtype=np.float32)

        lower_bounds = []
        upper_bounds = []
        for joint_name in self._joint_names:
            joint_limit = self._moveit_robot._r.get_joint_limits(joint_name)[0]
            if self.control_mode == 'position':
                lower_bounds.append(joint_limit[0])
                upper_bounds.append(joint_limit[1])
            else:
                raise ValueError('Control mode {} is not known!'.format(self.control_mode))
        lower_bounds = np.asarray(lower_bounds)
        upper_bounds = np.asarray(upper_bounds)

        # no gripper now
        self._action_space = gym.spaces.Box(lower_bounds, upper_bounds, dtype=np.float32)

    def safety_check(self):
        """If robot is in safe state

        Arguments
        ----------

        Returns
        ----------
        - is_safe: bool
            True when the robot is safe

        """
        rs = moveit_msgs.msg.RobotState()
        for joint_name, joint_position in zip(self._joint_names, self._moveit_group.get_current_joint_values()):
            rs.joint_state.name.append(joint_name)
            rs.joint_state.position.append(joint_position)
        result = self._sv.get_state_validity(rs, self.group_name)

        is_safe = result.valid
        return is_safe

    def safety_predict(self, joint_values):
        """Will robot be in safe state

        Arguments
        ----------
        - joint_values: {str: float}
            Dictionary of joint name and the corresponding positions

        Returns
        ----------
        - is_safe: bool
            True when the robot is safe.

        """
        rs = moveit_msgs.msg.RobotState()
        for joint_name, joint_position in joint_values.items():
            rs.joint_state.name.append(joint_name)
            rs.joint_state.position.append(joint_position)
        result = self._sv.get_state_validity(rs, self.group_name)

        is_safe = result.valid
        return is_safe

    # @property
    # def enabled(self):
    #     """
    #     If robot is enabled.
    #     :return: if robot is enabled.
    #     """
    #     return intera_interface.RobotEnable(
    #         intera_interface.CHECK_VERSION).state().enabled

    def _set_joint_positions(self, joint_position_cmds):
        """Set joint position command

        Arguments
        ----------
        - joint_position_cmds: np.ndarray
            Joint position commands

        Returns
        ----------

        """
        # limit joint angles cmd
        current_joint_positions = self._moveit_group.get_current_joint_values()
        for joint_name, current_joint_position in zip(self._joint_names, current_joint_positions):
            joint_position_delta = joint_position_cmds[joint_name] - current_joint_position
            joint_position_cmds[joint_name] = current_joint_position + joint_position_delta * POS_DELTA_FACTOR

        if self.safety_predict(joint_position_cmds):
            self._moveit_group.go(joint_position_cmds, wait=True)

    # def _set_limb_joint_velocities(self, joint_angle_cmds):
    #     self._limb.set_joint_velocities(joint_angle_cmds)
    #
    # def _set_limb_joint_torques(self, joint_angle_cmds):
    #     self._limb.set_joint_torques(joint_angle_cmds)
    #
    # def _set_gripper_position(self, position):
    #     # no gripper now
    #     self._gripper.set_position(position)

    def _move_to_start_position(self):
        if rospy.is_shutdown():
            return
        joint_value_target = []
        for joint_name in self._joint_names:
            joint_value_target.append(self.initial_joint_pos[joint_name])
        self._moveit_group.go(joint_value_target, wait=True)

    def reset(self):
        """Reset UR5

        Arguments
        ----------

        Returns
        ----------

        """
        self._move_to_start_position()

    def get_observation(self):
        """Get robot observation.

        Arguments
        ----------

        Returns
        ----------
        - obs: np.ndarray
            Current observation

        """
        # cartesian space
        ee_pose = self._moveit_group.get_current_pose()
        ee_position = np.array([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z])
        ee_orientation = np.array([ee_pose.pose.orientation.x, ee_pose.pose.orientation.y,
                                   ee_pose.pose.orientation.z, ee_pose.pose.orientation.w])

        # joint space
        jac = np.asarray(self._moveit_group.get_jacobian_matrix(self._moveit_group.get_current_joint_values()))
        js = rospy.wait_for_message(self.js_topic_name, JointState)
        joint_positions = np.asarray(js.position)
        joint_velocities = np.asarray(js.velocity)
        ee_twist = np.matmul(jac, joint_velocities)

        obs = np.concatenate([ee_position, ee_orientation, ee_twist, joint_positions, joint_velocities], axis=0)

        return obs

    def send_command(self, commands):
        """Send command to UR5

        Arguments
        ----------
        - commands: np.ndarray
            Command for different joints and gripper

        Returns
        ----------

        """
        action_space = self.action_space
        commands = np.clip(commands, action_space.low, action_space.high)
        joint_commands = {}
        for joint_name, joint_command in zip(self._joint_names, commands[:len(self._joint_names)]):
            joint_commands[joint_name] = joint_command

        if self.control_mode == 'position':
            self._set_joint_positions(joint_commands)
        else:
            raise ValueError('Control mode {} is not known!'.format(self.control_mode))
        # elif self.control_mode == 'velocity':
        #     self._set_limb_joint_velocities(joint_commands)
        # elif self.control_mode == 'effort':
        #     self._set_limb_joint_torques(joint_commands)

        # no gripper now
        # self._set_gripper_position(commands[7])

    # @property
    # def gripper_pose(self):
    #     """
    #     Get the gripper pose.
    #     :return: gripper pose
    #     """
    #     return self._limb.endpoint_pose()

    @property
    def observation_space(self):
        """Observation space

        Arguments
        ----------

        Returns
        ----------
        - observation_space: gym.spaces
            Observation space

        """
        return self._observation_space

    @property
    def action_space(self):
        """Action space

        Arguments
        ----------

        Returns
        ----------
        - action_space: gym.spaces
            Action space

        """
        return self._action_space
