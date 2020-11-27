import rospy
import mujoco_ros_msgs.msg
from mujoco_ros_msgs.srv import SetJointQPos, SetJointQPosRequest, SetOptGeomGroup, SetOptGeomGroupRequest, \
    SetFixedCamera, SetFixedCameraRequest, SetCtrl, SetCtrlRequest
from std_srvs.srv import Trigger, TriggerRequest
import moveit_commander
import actionlib
import moveit_msgs.msg
import control_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import jog_msgs.msg

import numpy as np

from robot.environments.ros_util import StateValidity


class MujocoROSError(Exception):
    """Base class for exceptions in MujocoROS class."""
    pass


class MujocoROS:
    def __init__(self, node_name="mujoco_ros", prefix="/mujoco_ros", manipulator_group_name="manipulator",
                 gripper_group_name="gripper", state_validity_srv="/check_state_validity",
                 jog_frame_topic="/jog_frame", jog_joint_topic="/jog_joint", joint_state_topic="/joint_states"):
        self.node_name = node_name
        self.prefix = prefix
        self.manipulator_group_name = manipulator_group_name
        self.gripper_group_name = gripper_group_name
        self.jog_frame_topic = jog_frame_topic
        self.jog_joint_topic = jog_joint_topic
        self.state_validity_srv = state_validity_srv
        self.joint_state_topic = joint_state_topic

        # ROS node
        rospy.init_node(self.node_name, anonymous=True)

        # moveit
        self._moveit_robot = moveit_commander.RobotCommander()
        # self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        # self._moveit_gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        self._moveit_manipulator_group = self._moveit_robot.get_group(self.manipulator_group_name)
        self._moveit_manipulator_group.allow_replanning(True)
        self._moveit_gripper_group = self._moveit_robot.get_group(self.gripper_group_name)
        self._moveit_gripper_group.allow_replanning(True)

        # joint trajectory controllers
        self._controllers = self._get_controllers()

        # jog
        # reference: https://github.com/tork-a/jog_control
        # self._jog_time_from_start = self._get_param("/jog/time_from_start")
        self._jog_group = self._get_param("/jog/group")
        self._jog_target_link = self._get_param("/jog/target_link")
        self._jog_base_link = self._get_param("/jog/base_link")
        self._jog_joint_names = self._get_param("/jog_joint_node/joint_names")

        # self._last_time = rospy.Time.now()
        # self._act_pos = None
        # self._act_quat = None

        self._jog_frame_pub = rospy.Publisher(self.jog_frame_topic, jog_msgs.msg.JogFrame, queue_size=1)
        self._jog_joint_pub = rospy.Publisher(self.jog_joint_topic, jog_msgs.msg.JogJoint, queue_size=1)

        # misc
        # try:
        #     self._joint_names = self._get_param("/robot_joints")
        # except MujocoROSError:
        #     self._joint_names = self._moveit_manipulator_group.get_active_joints()
        self._arm_joint_names = self._moveit_manipulator_group.get_active_joints()
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

    def _get_controllers(self):
        controller_params = self._get_param("/move_group/controller_list")
        controllers = {}
        for controller_param in controller_params:
            if "name" not in controller_param:
                raise MujocoROSError("Name must be specified for each controller!")

            if "joints" not in controller_param:
                raise MujocoROSError("Joints must be specified for each controller!")

            if "action_ns" in controller_param:
                action_ns = controller_param["action_ns"]
            else:
                action_ns = ""

            if not isinstance(controller_param["joints"], list):
                raise MujocoROSError("Joints for controller {} is not specified as an array".format(
                    controller_param["name"]))

            joints = controller_param["joints"]

            if "type" in controller_param:
                ctrl_type = controller_param['type']
            else:
                ctrl_type = "FollowJointTrajectory"

            if ctrl_type != "FollowJointTrajectory":
                raise MujocoROSError("Controller type {} is not supported".format(ctrl_type))

            controllers[controller_param["name"]] = {
                "action_ns": action_ns,
                "joints": joints,
                "traj_pub": rospy.Publisher(
                    controller_param["name"] + "/command",
                    trajectory_msgs.msg.JointTrajectory, queue_size=10),
                "traj_client": actionlib.SimpleActionClient(
                    controller_param["name"] + "/" + action_ns, control_msgs.msg.FollowJointTrajectoryAction)}
            if controllers[controller_param["name"]]["traj_client"].wait_for_server(rospy.Duration(3)):
                rospy.loginfo("{} is ready.".format(controller_param["name"] + "/" + action_ns))
            else:
                raise MujocoROSError("Get trajectory controller service failed: {}".format(action_ns))

        return controllers

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
        joint_limits = []
        for joint_name in joint_names:
            # joint_limit = self._moveit_robot._r.get_joint_limits(joint_name)[0]
            joint_limit = self._moveit_robot.get_joint(joint_name).bounds()
            if joint_type == 'pos':
                joint_limits.append(joint_limit)
            else:
                raise ValueError("Joint type \"{}\" is unknown!".format(joint_type))
        joint_limits = np.array(joint_limits)

        return joint_limits

    def get_joint_pos(self, joint_names, ros=True):
        joint_states_msg = None
        if ros:
            try:
                joint_states_msg = rospy.wait_for_message(self.joint_state_topic, sensor_msgs.msg.JointState, 3)
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
                joint_states_msg = rospy.wait_for_message(self.joint_state_topic, sensor_msgs.msg.JointState, 3)
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

    def get_eef_vel(self):
        joint_values = self.get_joint_pos(self._arm_joint_names)
        jac_mat = self._moveit_manipulator_group.get_jacobian_matrix(joint_values)
        index_map = dict((name, idx) for idx, name in enumerate(self._moveit_manipulator_group.get_active_joints()))
        index = [index_map[name] for name in self._arm_joint_names]
        jac_mat = jac_mat[:, index]  # switch columns

        joint_velocities = self.get_joint_vel(self._arm_joint_names)
        eef_vel = np.matmul(jac_mat, joint_velocities)

        return eef_vel

    def get_eef_ori_vel(self):
        pass

    def get_object_pos(self, object_name):
        # TODO (chongyi zheng): object pose estimation using computer vision algorithm
        body_states_msg = None
        try:
            body_states_msg = rospy.wait_for_message(self.prefix + '/body_states',
                                                     mujoco_ros_msgs.msg.BodyStates, 3)
        except rospy.ROSException as e:
            MujocoROSError("Message read failed: {}".format(e))
        index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        idx = index_map[object_name]
        object_pos = [body_states_msg.pose[idx].position.x, body_states_msg.pose[idx].position.y,
                      body_states_msg.pose[idx].position.z]

        return object_pos

    def get_object_quat(self, object_name, format="xyzw"):
        # TODO (chongyi zheng): object pose estimation using computer vision algorithm
        body_states_msg = None
        try:
            body_states_msg = rospy.wait_for_message(self.prefix + '/body_states',
                                                     mujoco_ros_msgs.msg.BodyStates, 3)
        except rospy.ROSException as e:
            MujocoROSError("Message read failed: {}".format(e))
        index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        idx = index_map[object_name]
        object_ori = body_states_msg.pose[idx].orientation

        if format == "xyzw":
            object_quat = [object_ori.x, object_ori.y, object_ori.z, object_ori.w]
        elif format == "wxyz":
            object_quat = [object_ori.w, object_ori.x, object_ori.y, object_ori.z]
        else:
            raise MujocoROSError("Invalid object quaternion format!")

        return object_quat

    def set_fixed_camera(self, camera_id):
        request = SetFixedCameraRequest(camera_id=camera_id)
        rospy.wait_for_service(self.prefix + '/set_fixed_camera', 3)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(self.prefix + "/set_fixed_camera", SetFixedCamera)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_joint_qpos(self, names, values):
        if not isinstance(names, (list, tuple)):
            names = [names]
            values = [values]

        request = SetJointQPosRequest()
        request.name = names
        for value in values:
            multi_array = std_msgs.msg.Float64MultiArray()
            multi_array.data = [value] if isinstance(value, (float, np.float)) else value
            request.value.append(multi_array)
        rospy.wait_for_service(self.prefix + '/set_joint_qpos', 3)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(self.prefix + "/set_joint_qpos", SetJointQPos)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_ctrl(self, names, ctrls):
        request = SetCtrlRequest(name=names, ctrl=ctrls)
        rospy.wait_for_service(self.prefix + '/set_ctrl', 3)
        try:
            set_ctrl_srv = rospy.ServiceProxy(self.prefix + "/set_ctrl", SetCtrl)
            set_ctrl_srv(request)
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

    def reset(self):
        request = TriggerRequest()
        rospy.wait_for_service(self.prefix + '/reset', 3)
        try:
            reset_srv = rospy.ServiceProxy(self.prefix + "/reset", Trigger)
            reset_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def is_position_valid(self, joint_positions):
        assert isinstance(joint_positions, dict) or isinstance(joint_positions, list) or \
               isinstance(joint_positions, np.ndarray), "Invalid joint positions to check!"

        rs = moveit_msgs.msg.RobotState()
        if isinstance(joint_positions, dict):
            rs.joint_state.name, rs.joint_state.position = list(joint_positions.keys()), list(joint_positions.values())
        else:
            rs.joint_state.name, rs.joint_state.position = self._arm_joint_names, list(joint_positions)
        result = self._state_validity.get_state_validity(rs, self.manipulator_group_name)

        is_valid = result.valid
        return is_valid

    # def goto_arm_positions(self, arm_joint_positions, wait=True):
    #     if self.is_position_valid(arm_joint_positions):
    #         # go to target joint positions
    #         self._moveit_manipulator_group.go(arm_joint_positions, wait=wait)
    #
    #         # calling `stop()` ensures that there is no residual movement
    #         # self._moveit_manipulator_group.stop()
    #     else:
    #         raise MujocoROSError("Invalid joint positions: {}".format(arm_joint_positions))

    def goto_eef_pose(self, eef_pos, eef_quat, quat_format="xyzw", wait=True):
        assert np.shape(eef_pos) == (3,) and np.shape(eef_quat) == (4,), "Invalid end effector pose!"

        pose = geometry_msgs.msg.Pose()
        pose.position.x = eef_pos[0]
        pose.position.y = eef_pos[1]
        pose.position.z = eef_pos[2]

        if quat_format == "xyzw":
            pose.orientation.x = eef_quat[0]
            pose.orientation.y = eef_quat[1]
            pose.orientation.z = eef_quat[2]
            pose.orientation.w = eef_quat[3]
        elif quat_format == "wxyz":
            pose.orientation.w = eef_quat[0]
            pose.orientation.x = eef_quat[1]
            pose.orientation.y = eef_quat[2]
            pose.orientation.z = eef_quat[3]

        # TODO (chongyi zheng): check validity of pose
        # go to target pose
        self._moveit_manipulator_group.set_pose_target(pose)
        self._moveit_manipulator_group.go(wait=wait)

        # calling `stop()` ensures that there is no residual movement
        # self._moveit_manipulator_group.stop()

        # it is always good to clear your targets after planning with poses.
        # self._moveit_manipulator_group.clear_pose_targets()

    # def move_eef_pose(self, eef_pos, eef_quat, quat_format="xyzw"):
    #     joint_states_msg = None
    #     try:
    #         joint_states_msg = rospy.wait_for_message(self.joint_state_topic, sensor_msgs.msg.JointState, 3)
    #     except rospy.ROSException as e:
    #         MujocoROSError("Message read failed: {}".format(e))
    #
    #     # fk_response = self._fk.get_fk(self._jog_target_link, joint_states_msg, self._jog_base_link)
    #     # if fk_response.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
    #     #     print("****FK {} error: {}".format(fk_response, fk_response.error_code.val))
    #     #     return
    #     #
    #     # if len(fk_response.pose_stamped) != 1:
    #     #     raise rospy.ServiceException("fk {} multiple poses!".format(fk_response))
    #
    #     # Apply jog
    #     ref_pose = geometry_msgs.msg.PoseStamped()
    #     ref_pose.header.frame_id = self._jog_base_link
    #     ref_pose.header.stamp = rospy.Time.now()
    #     ref_pose.pose.position.x = eef_pos[0]
    #     ref_pose.pose.position.y = eef_pos[1]
    #     ref_pose.pose.position.z = eef_pos[2]
    #
    #     if quat_format == "xyzw":
    #         ref_pose.pose.orientation.x = eef_quat[0]
    #         ref_pose.pose.orientation.y = eef_quat[1]
    #         ref_pose.pose.orientation.z = eef_quat[2]
    #         ref_pose.pose.orientation.w = eef_quat[3]
    #     elif quat_format == "wxyz":
    #         ref_pose.pose.orientation.w = eef_quat[0]
    #         ref_pose.pose.orientation.x = eef_quat[1]
    #         ref_pose.pose.orientation.y = eef_quat[2]
    #         ref_pose.pose.orientation.z = eef_quat[3]
    #
    #     # Solve inverse kinematics
    #     ik_response = self._ik.get_ik(self._jog_group, self._jog_target_link, joint_states_msg, ref_pose)
    #     if ik_response.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
    #         print("****IK error: {}".format(ik_response.error_code.val))
    #         return
    #
    #     solution_joint_states = ik_response.solution.joint_state
    #     index_map = dict((name, idx) for idx, name in enumerate(solution_joint_states.name))
    #     solution_joint_positions = [solution_joint_states.position[index_map[name]] for name in joint_states_msg.name]
    #     max_err = np.max(np.array(solution_joint_positions) - np.array(joint_states_msg.position))
    #     if max_err > np.pi / 2:
    #         print("**** Validation check Failed: {}".format(max_err))
    #         return
    #     elif max_err < 1e-3:
    #         return
    #
    #     # Publish trajectory message for each controller
    #     for controller_key, controller_val in self._jog_controllers.items():
    #         point = trajectory_msgs.msg.JointTrajectoryPoint()
    #         point.positions = [solution_joint_states.position[index_map[name]] for name in controller_val["joints"]]
    #         point.velocities = []
    #         point.accelerations = []
    #         point.time_from_start = rospy.Duration(self._jog_time_from_start)
    #
    #         traj = trajectory_msgs.msg.JointTrajectory()
    #         traj.header.stamp = rospy.Time.now()
    #         traj.header.frame_id = self._jog_base_link
    #         traj.joint_names = controller_val["joints"]
    #         traj.points.append(point)
    #
    #         controller_val["traj_pub"].publish(traj)

    def jog_eef_pose(self, linear_delta, angular_delta, avoid_collisions=True):
        """Send jog message directly"""

        # if (rospy.Time.now() - self._jog_frame_msg.header.stamp).to_sec() > 0.1:
        jog_frame_msg = jog_msgs.msg.JogFrame()
        jog_frame_msg.header.stamp = rospy.Time.now()
        jog_frame_msg.header.frame_id = self._jog_base_link
        jog_frame_msg.group_name = self._jog_group
        jog_frame_msg.link_name = self._jog_target_link
        jog_frame_msg.linear_delta.x = linear_delta[0]
        jog_frame_msg.linear_delta.y = linear_delta[1]
        jog_frame_msg.linear_delta.z = linear_delta[2]
        jog_frame_msg.angular_delta.x = angular_delta[0]
        jog_frame_msg.angular_delta.y = angular_delta[1]
        jog_frame_msg.angular_delta.z = angular_delta[2]
        jog_frame_msg.avoid_collisions = avoid_collisions

        # Publish only if the all command are not equal zero
        # Not good, we need to compare slider value by some way...
        if jog_frame_msg.linear_delta.x != 0 or jog_frame_msg.linear_delta.y != 0 or \
            jog_frame_msg.linear_delta.z != 0 or jog_frame_msg.angular_delta.x != 0 or \
            jog_frame_msg.angular_delta.y != 0 or jog_frame_msg.angular_delta.z != 0:
            self._jog_frame_pub.publish(jog_frame_msg)

        # joint_states_msg = None
        # try:
        #     joint_states_msg = rospy.wait_for_message(self.joint_state_topic, sensor_msgs.msg.JointState, 3)
        # except rospy.ROSException as e:
        #     MujocoROSError("Message read failed: {}".format(e))
        #
        # if rospy.Time.now() > self._last_time + rospy.Duration(self._jog_time_from_start):
        #     fk_response = self._fk.get_fk(self._jog_target_link, joint_states_msg, self._jog_base_link)
        #     if fk_response.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        #         print("****FK error: {}".format(fk_response.error_code.val))
        #         return
        #
        #     if len(fk_response.pose_stamped) != 1:
        #         raise rospy.ServiceException("fk {} multiple poses!".format(fk_response))
        #
        #     self._act_pos = np.array([fk_response.pose_stamped[0].pose.position.x,
        #                               fk_response.pose_stamped[0].pose.position.y,
        #                               fk_response.pose_stamped[0].pose.position.z])
        #     self._act_quat = np.array([fk_response.pose_stamped[0].pose.orientation.x,
        #                                fk_response.pose_stamped[0].pose.orientation.y,
        #                                fk_response.pose_stamped[0].pose.orientation.z,
        #                                fk_response.pose_stamped[0].pose.orientation.w])
        #     # self._pose = np.concatenate([self.get_eef_pos(), self.get_eef_quat()])
        #
        # # Apply jog
        # # self._act_pos = self.get_eef_pos()
        # # self._act_quat = self.get_eef_quat()
        # # act_pos = np.array([-0.23143887665151813, -0.05311485741178669, 0.929943799921143])
        # # act_quat = np.array([0.6914297083222615, -0.722388842430885, 0.0042093448984817875, 0.007848579252590351])
        # # self._act_pos.append(self.get_eef_pos())
        # # self._act_quat.append(self.get_eef_quat())
        # # act_pos = np.mean(self._act_pos, axis=0)
        # # act_quat = np.mean(self._act_quat, axis=0)
        # # act_pos = self.get_eef_pos("gripper0_grip_site")
        # # act_quat = self.get_eef_quat("robot0_right_hand")
        #
        # ref_pose = geometry_msgs.msg.PoseStamped()
        # ref_pose.header.frame_id = self._jog_base_link
        # ref_pose.header.stamp = rospy.Time.now()
        # ref_pos = self._act_pos + linear_delta
        # ref_pose.pose.position.x = ref_pos[0]
        # ref_pose.pose.position.y = ref_pos[1]
        # ref_pose.pose.position.z = ref_pos[2]
        #
        # # act_quat = np.array([act_pose.orientation.x, act_pose.orientation.y, act_pose.orientation.z,
        # #                      act_pose.orientation.w])
        # jog_quat = T.axisangle2quat(angular_delta)
        # ref_quat = T.quat_multiply(jog_quat, self._act_quat)  # xyzw
        # ref_pose.pose.orientation.x = ref_quat[0]
        # ref_pose.pose.orientation.y = ref_quat[1]
        # ref_pose.pose.orientation.z = ref_quat[2]
        # ref_pose.pose.orientation.w = ref_quat[3]
        #
        # # Solve inverse kinematics
        # ik_response = self._ik.get_ik(self._jog_group, self._jog_target_link, joint_states_msg, ref_pose)
        # if ik_response.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        #     print("****IK error: {}".format(ik_response.error_code.val))
        #     return
        #
        # solution_joint_states = ik_response.solution.joint_state
        # state_index_map = dict((name, idx) for idx, name in enumerate(joint_states_msg.name))
        # solution_index_map = dict((name, idx) for idx, name in enumerate(solution_joint_states.name))
        # solution_joint_positions = [solution_joint_states.position[solution_index_map[name]]
        #                             for name in joint_states_msg.name]
        # max_err = np.max(np.array(solution_joint_positions) - np.array(joint_states_msg.position))
        # if max_err > np.pi / 2:
        #     print("**** Validation check Failed: {}".format(max_err))
        #     return
        # # elif max_err < 1e-3:
        # #     return
        #
        # # Publish trajectory message for each controller
        # for controller_key, controller_val in self._jog_controllers.items():
        #     point = trajectory_msgs.msg.JointTrajectoryPoint()
        #     solution_joint_positions = np.array([solution_joint_states.position[solution_index_map[name]]
        #                                          for name in controller_val["joints"]])
        #     state_joint_positions = np.array([joint_states_msg.position[state_index_map[name]]
        #                                       for name in controller_val["joints"]])
        #     mask = (solution_joint_positions - state_joint_positions) > 1e-3
        #     # solution_joint_positions[mask] = state_joint_positions[mask]
        #     # solution_joint_positions = solution_joint_positions[mask]
        #     # joint_names = np.array(controller_val["joints"])[mask].tolist()
        #     point.positions = solution_joint_positions.tolist()
        #     # point.velocities = [solution_joint_states.velocity[index_map[name]] for name in controller_val["joints"]]
        #     point.velocities = []
        #     point.accelerations = []
        #     point.time_from_start = rospy.Duration(self._jog_time_from_start)
        #
        #     traj = trajectory_msgs.msg.JointTrajectory()
        #     traj.header.stamp = rospy.Time.now()
        #     traj.header.frame_id = self._jog_base_link
        #     traj.joint_names = controller_val["joints"]
        #     # traj.joint_names = joint_names
        #     traj.points.append(point)
        #
        #     controller_val["traj_pub"].publish(traj)
        #
        # self._act_pos = ref_pos
        # self._act_quat = ref_quat

    def goto_arm_positions(self, arm_joint_positions, time_from_start=4.0, wait=False):
        if self.is_position_valid(arm_joint_positions):
            for controller_key, controller_val in self._controllers.items():
                if 'manipulator' in controller_key:
                    index_maps = dict((name, idx) for idx, name in enumerate(arm_joint_positions.keys()))
                    indices = [index_maps[joint_name] for joint_name in controller_val["joints"]]

                    point = trajectory_msgs.msg.JointTrajectoryPoint()
                    point.positions = np.array(list(arm_joint_positions.values()))[indices].tolist()
                    point.velocities = []
                    point.accelerations = []
                    point.time_from_start = rospy.Duration().from_sec(time_from_start)

                    goal = control_msgs.msg.FollowJointTrajectoryGoal()
                    goal.trajectory.header.stamp = rospy.Time.now()
                    goal.trajectory.joint_names = controller_val["joints"]
                    goal.trajectory.points.append(point)

                    if wait:
                        controller_val["traj_client"].send_goal_and_wait(goal)
                    else:
                        controller_val["traj_client"].send_goal(goal)
        else:
            raise MujocoROSError("Invalid joint positions: {}".format(arm_joint_positions))

    def goto_gripper_positions(self, gripper_joint_positions, time_from_start=1.0):
        # jog_joint_msg = jog_msgs.msg.JogJoint()
        # jog_joint_msg.header.stamp = rospy.Time.now()
        # jog_joint_msg.joint_names = self._jog_joint_names
        # # index of 'gripper0_finger_joint' = 0
        # deltas = np.zeros(len(self._jog_joint_names))
        # index_maps = dict((name, idx) for idx, name in enumerate(self._jog_joint_names))
        # indices = [index_maps[gripper_joint_name] for gripper_joint_name in gripper_joint_deltas]
        # deltas[indices] = np.array(list(gripper_joint_deltas.values()))
        # jog_joint_msg.deltas = deltas.tolist()
        #
        # if np.any(deltas):
        #     self._jog_joint_pub.publish(jog_joint_msg)

        # Publish trajectory message for each controller
        for controller_key, controller_val in self._controllers.items():
            if 'gripper' in controller_key:
                index_maps = dict((name, idx) for idx, name in enumerate(gripper_joint_positions.keys()))
                indices = [index_maps[joint_name] for joint_name in controller_val["joints"]]

                point = trajectory_msgs.msg.JointTrajectoryPoint()
                point.positions = np.array(list(gripper_joint_positions.values()))[indices].tolist()
                point.velocities = []
                point.accelerations = []
                point.time_from_start = rospy.Duration().from_sec(time_from_start)

                traj = trajectory_msgs.msg.JointTrajectory()
                traj.header.stamp = rospy.Time.now()
                traj.joint_names = controller_val["joints"]
                traj.points.append(point)

                controller_val["traj_pub"].publish(traj)

    # def goto_gripper_positions(self, gripper_joint_positions, wait=True):
    #     if self.is_position_valid(gripper_joint_positions):
    #         # go to target joint positions
    #         self._moveit_gripper_group.go(gripper_joint_positions, wait=wait)
    #
    #         # calling `stop()` ensures that there is no residual movement
    #         # self._moveit_gripper_group.stop()
    #     else:
    #         raise MujocoROSError("Invalid joint positions: {}".format(gripper_joint_positions))

    # def open_gripper(self, wait=True):
    #     target_values = self._moveit_gripper_group.get_named_target_values('open')
    #     self._moveit_gripper_group.go(target_values, wait=wait)

    # def close_gripper(self, wait=True):
    #     target_values = self._moveit_gripper_group.get_named_target_values('close')
    #     self._moveit_gripper_group.go(target_values, wait=wait)
