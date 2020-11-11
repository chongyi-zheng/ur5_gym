import rospy
import mujoco_ros_msgs.msg
from mujoco_ros_msgs.srv import SetJointQPos, SetJointQPosRequest, SetOptGeomGroup, SetOptGeomGroupRequest, \
    SetFixedCamera, SetFixedCameraRequest
import moveit_msgs.msg
import moveit_commander
import sensor_msgs.msg

import numpy as np

from robot.environments.ros_util import StateValidity


class MujocoROSError(Exception):
    """Base class for exceptions in MujocoROS class."""
    pass


class MujocoROS:
    def __init__(self, node_name='mujoco_ros', prefix='/mujoco_ros', manipulator_group_name='manipulator',
                 gripper_group_name='gripper', state_validity_srv='/check_state_validity',
                 joint_state_msg='/joint_states'):
        self.node_name = node_name
        self.prefix = prefix
        self.manipulator_group_name = manipulator_group_name
        self.gripper_group_name = gripper_group_name
        self.state_validity_srv = state_validity_srv
        self.joint_state_msg = joint_state_msg

        rospy.init_node(self.node_name, anonymous=True)
        self._moveit_robot = moveit_commander.RobotCommander()
        # self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        # self._moveit_gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        self._moveit_manipulator_group = self._moveit_robot.get_group(self.manipulator_group_name)
        self._moveit_gripper_group = self._moveit_robot.get_group(self.gripper_group_name)

        try:
            self._joint_names = self._get_param("/robot_joints")
        except MujocoROSError:
            self._joint_names = self._moveit_manipulator_group.get_active_joints()
        self._state_validity = StateValidity(self.state_validity_srv)

    def _get_param(self, param_name, selections=None):
        try:
            values = rospy.get_param(param_name)
        except KeyError:
            raise MujocoROSError("{} is not found on ROS parameter server!".format(param_name))

        if selections is not None:
            if len(selections) > 1:
                values = [values[selection] for selection in selections]
            elif len(selections) == 1:
                values = [values[selections[0]]]
            else:  # empty
                values = []

        return values

    @property
    def timestep(self):
        param_name = self.prefix + '/timestep'
        return self._get_param(param_name)

    @property
    def ncam(self):
        param_name = self.prefix + '/ncam'
        return self._get_param(param_name)

    def joint_pos_indexes(self, robot_joints):
        param_name = self.prefix + '/joint_pos_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def joint_vel_indexes(self, robot_joints):
        param_name = self.prefix + '/joint_vel_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def actuator_name2id(self, actuator_names):
        param_name = self.prefix + '/actuator_name2id'
        if isinstance(actuator_names, str):
            actuator_names = [actuator_names]

        selected_indexes = self._get_param(param_name, selections=actuator_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def camera_name2id(self, camera_names):
        param_name = self.prefix + '/camera_name2id'
        if isinstance(camera_names, str):
            camera_names = [camera_names]

        selected_indexes = self._get_param(param_name, selections=camera_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def joint_name2id(self, joint_names):
        param_name = self.prefix + '/joint_name2id'
        if isinstance(joint_names, str):
            joint_names = [joint_names]

        selected_indexes = self._get_param(param_name, selections=joint_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def body_name2id(self, body_names):
        param_name = self.prefix + '/body_name2id'
        if isinstance(body_names, str):
            body_names = [body_names]

        selected_indexes = self._get_param(param_name, selections=body_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def geom_name2id(self, geom_names):
        param_name = self.prefix + '/geom_name2id'
        if isinstance(geom_names, str):
            geom_names = [geom_names]

        selected_indexes = self._get_param(param_name, selections=geom_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def geom_id2name(self, geom_ids):
        param_name = self.prefix + '/geom_ids'
        if isinstance(geom_ids, str):
            geom_ids = [geom_ids]

        selected_names = self._get_param(param_name, selections=geom_ids)

        if len(selected_names) == 1:
            return selected_names[0]
        else:
            return selected_names

    def site_name2id(self, site_names):
        param_name = self.prefix + '/site_name2id'
        if isinstance(site_names, str):
            site_names = [site_names]

        selected_indexes = self._get_param(param_name, selections=site_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def get_joint_limits(self, joint_names, joint_type='pos'):
        low_bounds = []
        high_bounds = []
        for joint_name in joint_names:
            # joint_limit = self._moveit_robot._r.get_joint_limits(joint_name)[0]
            joint_limits = self._moveit_robot.get_joint(joint_name).bounds()
            if joint_type == 'pos':
                low_bounds.append(joint_limits[0])
                high_bounds.append(joint_limits[1])
            else:
                raise ValueError("Joint type \"{}\" is unknown!".format(joint_type))
        low_bounds = np.asarray(low_bounds)
        high_bounds = np.asarray(high_bounds)

        return low_bounds, high_bounds

    def get_joint_pos(self, joint_names, ros=True):
        joint_states_msg = None
        if ros:
            try:
                joint_states_msg = rospy.wait_for_message(self.joint_state_msg, sensor_msgs.msg.JointState, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
        else:
            try:
                joint_states_msg = rospy.wait_for_message(self.prefix + '/joint_states',
                                                          mujoco_ros_msgs.msg.JointStates, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
        index_map = dict((name, idx) for idx, name in enumerate(joint_states_msg.name))
        indices = [index_map[name] for name in joint_names]
        joint_pos = []
        for idx in indices:
            if not ros:
                if len(joint_states_msg.position[idx].data) == 1:
                    joint_pos.append(joint_states_msg.position[idx].data[0])
                else:
                    joint_pos.append(list(joint_states_msg.position[idx].data))
            else:
                joint_pos.append(joint_states_msg.position[idx])

        return joint_pos

    def get_joint_vel(self, joint_names, ros=True):
        joint_states_msg = None
        if ros:
            try:
                joint_states_msg = rospy.wait_for_message(self.joint_state_msg, sensor_msgs.msg.JointState, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
        else:
            try:
                joint_states_msg = rospy.wait_for_message(self.prefix + '/joint_states',
                                                          mujoco_ros_msgs.msg.JointStates, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
        index_map = dict((name, idx) for idx, name in enumerate(joint_states_msg.name))
        indices = [index_map[name] for name in joint_names]
        joint_vel = []
        for idx in indices:
            if not ros:
                if len(joint_states_msg.velocity[idx].data) == 1:
                    joint_vel.append(joint_states_msg.velocity[idx].data[0])
                else:
                    joint_vel.append(list(joint_states_msg.velocity[idx].data))
            else:
                joint_vel.append(joint_states_msg.velocity[idx])

        return joint_vel

    def get_eef_pos(self, eef_site_name=None):
        if eef_site_name is None:  # read from moveit
            eef_pose = self._moveit_manipulator_group.get_current_pose()
            eef_pos = [eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z]
        else:  # read from mujoco
            site_states_msg = None
            try:
                site_states_msg = rospy.wait_for_message(self.prefix + '/site_states',
                                                         mujoco_ros_msgs.msg.SiteStates, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
            index_map = dict((name, idx) for idx, name in enumerate(site_states_msg.name))
            idx = index_map[eef_site_name]
            eef_pos = list(site_states_msg.position[idx].data)

        return eef_pos

    def get_eef_quat(self, eef_body_name=None, format="xyzw"):
        if eef_body_name is None:  # read from moveit
            eef_pose = self._moveit_manipulator_group.get_current_pose()
            eef_pose_ori = eef_pose.pose.orientation
        else:
            body_states_msg = None
            try:
                body_states_msg = rospy.wait_for_message(self.prefix + '/body_states',
                                                         mujoco_ros_msgs.msg.BodyStates, 3)
            except rospy.ROSException as e:
                MujocoROSError("Message read failed: {}".format(e))
            index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
            idx = index_map[eef_body_name]
            eef_pose_ori = body_states_msg.pose[idx].orientation

        if format == "xyzw":
            eef_quat = [eef_pose_ori.x, eef_pose_ori.y, eef_pose_ori.z, eef_pose_ori.w]
        elif format == "wxyz":
            eef_quat = [eef_pose_ori.w, eef_pose_ori.x, eef_pose_ori.y, eef_pose_ori.z]
        else:
            raise MujocoROSError("Invalid end effector quaternion format!")

        return eef_quat

    def set_fixed_camera(self, camera_id):
        request = SetFixedCameraRequest(camera_id=camera_id)
        rospy.wait_for_service(self.prefix + '/set_fixed_camera', 3)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(self.prefix + "/set_fixed_camera", SetFixedCamera)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_joint_qpos(self, name, value):
        request = SetJointQPosRequest(name=name, value=value)
        rospy.wait_for_service(self.prefix + '/set_joint_qpos', 3)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(self.prefix + "/set_joint_qpos", SetJointQPos)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_vopt_geomgroup(self, index, value):
        request = SetOptGeomGroupRequest(index=index, value=value)
        rospy.wait_for_service(self.prefix + '/set_vopt_geomgroup', 3)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(self.prefix + "/set_vopt_geomgroup", SetOptGeomGroup)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def is_position_valid(self, joint_positions):
        assert isinstance(joint_positions, dict) or isinstance(joint_positions, list) or \
               isinstance(joint_positions, np.ndarray), "Invalid joint positions to check!"

        rs = moveit_msgs.msg.RobotState()
        if isinstance(joint_positions, dict):
            rs.joint_state.name, rs.joint_state.position = list(joint_positions.keys()), list(joint_positions.values())
        else:
            rs.joint_state.name, rs.joint_state.position = self._joint_names, list(joint_positions)
        result = self._state_validity.get_state_validity(rs, self.manipulator_group_name)

        is_valid = result.valid
        return is_valid

    def goto_joint_positions(self, joint_positions):
        if self.is_position_valid(joint_positions):
            self._moveit_manipulator_group.go(joint_positions, wait=True)

    def open_gripper(self):
        target_values = self._moveit_gripper_group.get_named_target_values('open')
        self._moveit_gripper_group.go(target_values, wait=True)

    def close_gripper(self):
        target_values = self._moveit_gripper_group.get_named_target_values('close')
        self._moveit_gripper_group.go(target_values, wait=True)
