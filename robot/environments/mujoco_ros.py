import rospy
from mujoco_ros_msgs.srv import SetJointQPos, SetJointQPosRequest, SetOptGeomGroup, SetOptGeomGroupRequest, \
    SetFixedCamera, SetFixedCameraRequest, SetCtrl, SetCtrlRequest, GetBodyStates, GetBodyStatesRequest, \
    GetJointStates, GetJointStatesRequest, GetSiteStates, GetSiteStatesRequest, GetContactsRequest, GetContacts
from std_srvs.srv import Trigger, TriggerRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, \
    ListControllers, ListControllersRequest
import actionlib
import jog_msgs.msg
import control_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg

import numpy as np


class MujocoROSError(Exception):
    """Base class for exceptions in MujocoROS class."""
    pass


class MujocoROS:
    def __init__(self, node_name="mujoco_ros", env_prefix="/env", simulator_prefix="/mujoco_ros",
                 manipulator_group_name="manipulator", gripper_group_name="gripper",
                 state_validity_srv="/check_state_validity", jog_frame_topic="/jog_frame",
                 joint_state_topic="/joint_states"):
        self.node_name = node_name
        self.env_prefix = env_prefix
        self.simulator_prefix = simulator_prefix
        # self.manipulator_group_name = manipulator_group_name
        # self.gripper_group_name = gripper_group_name
        self.jog_frame_topic = jog_frame_topic
        # self.state_validity_srv = state_validity_srv
        # self.joint_state_topic = joint_state_topic

        # self._moveit_robot = None
        # self._moveit_manipulator_group = None
        # self._arm_joint_names = None
        self._controllers = None
        self._cartesian_controller_names = None
        self._manipulator_controller_names = None
        self._cartesian_target_pub = None
        self._robot_base_link = None
        self._jog_frame_pub = None
        # self._jog_joint_pub = None

        # ROS node
        rospy.init_node(self.node_name)

        # # moveit
        # self._moveit_robot = moveit_commander.RobotCommander()
        # # self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        # # self._moveit_gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        # self._moveit_manipulator_group = self._moveit_robot.get_group(self.manipulator_group_name)
        # self._moveit_manipulator_group.allow_replanning(True)
        # self._moveit_gripper_group = self._moveit_robot.get_group(self.gripper_group_name)
        # self._moveit_gripper_group.allow_replanning(True)
        # moveit_commander.roscpp_initialize(sys.argv)
        # self._moveit_robot = moveit_commander.RobotCommander()
        # self._moveit_manipulator_group = self._moveit_robot.get_group(self.manipulator_group_name)
        # self._arm_joint_names = self._moveit_manipulator_group.get_active_joints()
        # self._moveit_robot = None
        # self._moveit_manipulator_group = None
        # self._arm_joint_names = None
        # self.reset_moveit()

        # joint trajectory controllers
        # self._controllers = None
        # self._controllers = self._get_controllers()
        # self.reset_controllers()

        # terminate first, then spawn robot
        self.terminate()
        self.spawn()

        # jog
        # reference: https://github.com/tork-a/jog_control
        # self._jog_time_from_start = self._get_param(self.env_prefix + "/jog/time_from_start")
        self._jog_group = self._get_param(self.env_prefix + "/jog/group")
        self._jog_target_link = self._get_param(self.env_prefix + "/jog/target_link")
        self._jog_base_link = self._get_param(self.env_prefix + "/jog/base_link")
        # self._jog_joint_names = self._get_param(self.env_prefix + "/jog_joint_node/joint_names")

        # self._last_time = rospy.Time.now()
        # self._act_pos = None
        # self._act_quat = None

        # self._jog_frame_pub = rospy.Publisher(self.jog_frame_topic, jog_msgs.msg.JogFrame, queue_size=1)
        # self._jog_joint_pub = rospy.Publisher(self.jog_joint_topic, jog_msgs.msg.JogJoint, queue_size=1)
        # self._jog_frame_srv = rospy.ServiceProxy(self.jog_frame_topic, JogFrameCmd)

        # state validity
        # try:
        #     self._joint_names = self._get_param("/robot_joints")
        # except MujocoROSError:
        #     self._joint_names = self._moveit_manipulator_group.get_active_joints()
        # self._state_validity = StateValidity(self.state_validity_srv)

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

    # def _get_controllers(self):
    #     time.sleep(0.5)
    #     controller_params = self._get_param(self.env_prefix + "/move_group/controller_list")
    #     controllers = {}
    #     for controller_param in controller_params:
    #         if "name" not in controller_param:
    #             raise MujocoROSError("Name must be specified for each controller!")
    #
    #         if self.env_prefix != "":
    #             controller_param['name'] = self.env_prefix + '/' + controller_param['name']
    #
    #         if "joints" not in controller_param:
    #             raise MujocoROSError("Joints must be specified for each controller!")
    #
    #         if "action_ns" in controller_param:
    #             action_ns = controller_param["action_ns"]
    #         else:
    #             action_ns = ""
    #
    #         if not isinstance(controller_param["joints"], list):
    #             raise MujocoROSError("Joints for controller {} is not specified as an array".format(
    #                 controller_param["name"]))
    #
    #         joints = controller_param["joints"]
    #
    #         if "type" in controller_param:
    #             ctrl_type = controller_param['type']
    #         else:
    #             ctrl_type = "FollowJointTrajectory"
    #
    #         if ctrl_type != "FollowJointTrajectory":
    #             raise MujocoROSError("Controller type {} is not supported".format(ctrl_type))
    #
    #         controllers[controller_param["name"]] = {
    #             "action_ns": action_ns,
    #             "joints": joints,
    #             "traj_pub": rospy.Publisher(
    #                 controller_param["name"] + "/command",
    #                 trajectory_msgs.msg.JointTrajectory, queue_size=10),
    #             "traj_client": actionlib.SimpleActionClient(
    #                 controller_param["name"] + "/" + action_ns, control_msgs.msg.FollowJointTrajectoryAction)}
    #         if controllers[controller_param["name"]]["traj_client"].wait_for_server(rospy.Duration(15)):
    #             rospy.loginfo("{} is ready.".format(controller_param["name"] + "/" + action_ns))
    #         else:
    #             raise MujocoROSError("Get trajectory controller service failed: {}".format(
    #                 controller_param["name"] + "/" + action_ns))
    #
    #     return controllers

    def _get_controllers(self):
        try:
            req = ListControllersRequest()
            list_controller_srv = rospy.ServiceProxy(
                self.env_prefix + "/controller_manager/list_controllers", ListControllers)
            list_controller_srv.wait_for_service(3)
            list_controller_res = list_controller_srv(req)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

        self._controllers = []
        self._cartesian_controller_names = []
        self._manipulator_controller_names = []
        for controller_state in list_controller_res.controller:
            controller = {
                'claimed_resources': controller_state.claimed_resources,
                'name': controller_state.name,
                'state': controller_state.state,
                'type': controller_state.type,
            }

            if 'JointTrajectoryController' in controller['type']:
                controller['traj_pub'] = rospy.Publisher(
                    self.env_prefix + '/' + controller['name'] + '/command',
                    trajectory_msgs.msg.JointTrajectory, queue_size=10)
                controller['traj_client'] = actionlib.SimpleActionClient(
                    self.env_prefix + '/' + controller['name'] + '/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction)

                if controller['state'] == 'running':
                    try:
                        controller['traj_client'].wait_for_server(rospy.Duration(10))
                        rospy.loginfo("{} is ready.".format(self.env_prefix + '/' + controller['name']))
                    except:
                        MujocoROSError("{} is not found on ROS parameter server!".format(
                            self.env_prefix + '/' + controller['name'] + '/follow_joint_trajectory'))

            if 'cartesian' in controller['name']:
                self._cartesian_controller_names.append(controller['name'])
            elif 'manipulator' in controller['name']:
                self._manipulator_controller_names.append(controller['name'])

            if 'cartesian_motion' in controller['name']:
                self._cartesian_target_pub = rospy.Publisher(
                    self.env_prefix + '/' + controller['name'] + '/target_frame',
                    geometry_msgs.msg.PoseStamped, queue_size=10)
                self._robot_base_link = self._get_param(
                    self.env_prefix + '/' + controller['name'] + '/robot_base_link')

            self._controllers.append(controller)

    # def _joint_state_callback(self, msg):
    #     with self._joint_states_msg_lock:
    #         self._joint_states_msg = msg
    #
    # def _raw_joint_state_callback(self, msg):
    #     with self._raw_joint_states_msg_lock:
    #         self._raw_joint_states_msg = msg
    #
    # def _site_state_callback(self, msg):
    #     with self._site_states_msg_lock:
    #         self._site_states_msg = msg
    #
    # def _body_state_callback(self, msg):
    #     with self._body_states_msg_lock:
    #         self._body_states_msg = msg

    @property
    def timestep(self):
        param_name = self.env_prefix + self.simulator_prefix + '/timestep'
        return self._get_param(param_name)

    @property
    def ncam(self):
        param_name = self.env_prefix + self.simulator_prefix + '/ncam'
        return self._get_param(param_name)

    def joint_pos_indexes(self, robot_joints):
        param_name = self.env_prefix + self.simulator_prefix + '/joint_pos_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def joint_vel_indexes(self, robot_joints):
        param_name = self.env_prefix + self.simulator_prefix + '/joint_vel_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def actuator_name2id(self, actuator_names):
        param_name = self.env_prefix + self.simulator_prefix + '/actuator_name2id'
        if isinstance(actuator_names, str):
            actuator_names = [actuator_names]

        selected_indexes = self._get_param(param_name, selections=actuator_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def camera_name2id(self, camera_names):
        param_name = self.env_prefix + self.simulator_prefix + '/camera_name2id'
        if isinstance(camera_names, str):
            camera_names = [camera_names]

        selected_indexes = self._get_param(param_name, selections=camera_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def joint_name2id(self, joint_names):
        param_name = self.env_prefix + self.simulator_prefix + '/joint_name2id'
        if isinstance(joint_names, str):
            joint_names = [joint_names]

        selected_indexes = self._get_param(param_name, selections=joint_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def body_name2id(self, body_names):
        param_name = self.env_prefix + self.simulator_prefix + '/body_name2id'
        if isinstance(body_names, str):
            body_names = [body_names]

        selected_indexes = self._get_param(param_name, selections=body_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def geom_name2id(self, geom_names):
        param_name = self.env_prefix + self.simulator_prefix + '/geom_name2id'
        if isinstance(geom_names, str):
            geom_names = [geom_names]

        selected_indexes = self._get_param(param_name, selections=geom_names)

        return selected_indexes

    def geom_id2name(self, geom_ids):
        param_name = self.env_prefix + self.simulator_prefix + '/geom_ids'
        if isinstance(geom_ids, str):
            geom_ids = [geom_ids]

        selected_names = self._get_param(param_name, selections=geom_ids)

        if len(selected_names) == 1:
            return selected_names[0]
        else:
            return selected_names

    def site_name2id(self, site_names):
        param_name = self.env_prefix + self.simulator_prefix + '/site_name2id'
        if isinstance(site_names, str):
            site_names = [site_names]

        selected_indexes = self._get_param(param_name, selections=site_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def spawn(self):
        # spawn robot
        request = TriggerRequest()
        try:
            spawn_sim_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/spawn_sim_environment", Trigger)
            spawn_sim_srv.wait_for_service(3)
            spawn_sim_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

        # initialize moveit python wrapper
        # moveit_commander.roscpp_initialize(sys.argv)

        # reset controller and moveit
        self._get_controllers()
        # self._moveit_robot = moveit_commander.RobotCommander()
        # self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name,
        #                                                                      wait_for_servers=20.0)
        # self._arm_joint_names = self._moveit_manipulator_group.get_active_joints()

        # jog frame and jog joint publishers
        self._jog_frame_pub = rospy.Publisher(self.env_prefix + self.jog_frame_topic, jog_msgs.msg.JogFrame,
                                              queue_size=10)
        # self._jog_joint_pub = rospy.Publisher(self.env_prefix + self.jog_joint_topic, jog_msgs.msg.JogJoint,
        #                                       queue_size=10)

    def terminate(self):
        if self._controllers is not None:
            self._jog_frame_pub.unregister()
            # self._jog_joint_pub.unregister()
            # moveit_commander.roscpp_shutdown()
            # self._cartesian_target_pub.unregister()

            for controller in self._controllers:
                if controller['type'] == 'JointTrajectoryController':
                    controller['traj_pub'].unregister()
                    controller['traj_client'].action_client.stop()

            self._jog_frame_pub = None
            # self._jog_joint_pub = None
            # self._moveit_robot = None
            # self._moveit_manipulator_group = None
            # self._arm_joint_names = None
            self._controllers = None
            self._cartesian_controller_names = None
            self._manipulator_controller_names = None

        request = TriggerRequest()
        try:
            terminate_sim_srv = rospy.ServiceProxy(self.env_prefix + self.simulator_prefix + "/terminate_sim", Trigger)
            terminate_sim_srv.wait_for_service(3)
            terminate_sim_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    # def reset_moveit(self):
    #     # # moveit
    #     # self._moveit_robot = moveit_commander.RobotCommander()
    #     # self._moveit_manipulator_group = self._moveit_robot.get_group(self.manipulator_group_name)
    #     # # self._moveit_manipulator_group.allow_replanning(True)
    #     # # self._moveit_gripper_group = self._moveit_robot.get_group(self.gripper_group_name)
    #     # # self._moveit_gripper_group.allow_replanning(True)
    #     # self._arm_joint_names = self._moveit_manipulator_group.get_active_joints()
    #     # TODO (chongyi zheng): Do we need this?
    #     try:
    #         # moveit_commander.roscpp_initialize(sys.argv)
    #         self._moveit_robot = moveit_commander.RobotCommander()
    #         self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name,
    #                                                                              wait_for_servers=20.0)
    #     except:
    #         self.terminate()
    #         self.spawn()
    #         # moveit_commander.roscpp_initialize(sys.argv)
    #         self._moveit_robot = moveit_commander.RobotCommander()
    #         self._moveit_manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name,
    #                                                                              wait_for_servers=20.0)
    #
    #     self._arm_joint_names = self._moveit_manipulator_group.get_active_joints()
    #
    #     return True

    # def reset_controllers(self):
    #     try:
    #         for controller_name, controller in self._controllers.items():
    #             controller["traj_client"].wait_for_server(rospy.Duration(10))
    #             rospy.loginfo("{} is ready.".format(controller_name + "/" + controller["action_ns"]))
    #     except:
    #         success = False
    #         while not success:
    #             try:
    #                 self.terminate()
    #                 self.spawn()
    #                 for controller_name, controller in self._controllers.items():
    #                     controller["traj_client"].wait_for_server(rospy.Duration(10))
    #                     rospy.loginfo("{} is ready.".format(controller_name + "/" + controller["action_ns"]))
    #                 success = True
    #             except:
    #                 success = False
    #
    #     return True

    def get_joint_limits(self, actuator_names):
        param_name = self.env_prefix + self.simulator_prefix + '/actuator_ctrlrange'
        selected_actuator_ctrlranges = np.array(self._get_param(param_name, selections=actuator_names))

        return selected_actuator_ctrlranges

    def get_joint_pos(self, joint_names):
        try:
            request = GetJointStatesRequest()
            get_joint_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_joint_states", GetJointStates)
            get_joint_states_srv.wait_for_service(3)
            joint_states = get_joint_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

        joint_pos = []
        for name in joint_names:
            idx = joint_states.name.index(name)
            if len(joint_states.position[idx].data) == 1:
                joint_pos.append(joint_states.position[idx].data[0])
            else:
                joint_pos.append(joint_states.position[idx].data)

        return joint_pos

    def get_joint_vel(self, joint_names):
        request = GetJointStatesRequest()
        try:
            get_joint_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_joint_states", GetJointStates)
            get_joint_states_srv.wait_for_service(3)
            joint_states = get_joint_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

        joint_vel = []
        for name in joint_names:
            idx = joint_states.name.index(name)
            if len(joint_states.velocity[idx].data) == 1:
                joint_vel.append(joint_states.velocity[idx].data[0])
            else:
                joint_vel.append(joint_states.velocity[idx].data)

        return joint_vel

    def get_eef_pos(self, eef_site_name=None):
        try:
            request = GetSiteStatesRequest()
            get_site_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_site_states", GetSiteStates)
            get_site_states_srv.wait_for_service(3)
            site_states = get_site_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))
        idx = site_states.name.index(eef_site_name)
        eef_pos = list(site_states.position[idx].data)

        return eef_pos

    def get_eef_ori_mat(self, eef_site_name=None):
        try:
            request = GetSiteStatesRequest()
            get_site_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_site_states", GetSiteStates)
            get_site_states_srv.wait_for_service(3)
            site_states = get_site_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))
        idx = site_states.name.index(eef_site_name)
        eef_ori_mat = np.array(site_states.rotation_matrix[idx].data).reshape([3, 3])

        return eef_ori_mat

    def get_eef_quat(self, eef_body_name=None, format="xyzw"):
        # if eef_body_name is None:  # read from moveit
        #     eef_pose = self._moveit_manipulator_group.get_current_pose()
        #     eef_pose_ori = eef_pose.pose.orientation
        # else:
        #     # body_states_msg = None
        #     # try:
        #     #     body_states_msg = rospy.wait_for_message(self.env_prefix + self.simulator_prefix + '/body_states',
        #     #                                              mujoco_ros_msgs.msg.BodyStates, 3)
        #     # except rospy.ROSException as e:
        #     #     MujocoROSError("Message read failed: {}".format(e))
        #     # body_states_msg = rospy.wait_for_message(
        #     self.env_prefix + self.simulator_prefix + '/body_states', mujoco_ros_msgs.msg.BodyStates, 5)
        #     # with self._body_states_msg_lock:
        #     #     body_states_msg = copy.deepcopy(self._body_states_msg)
        #     request = GetBodyStatesRequest()
        #     try:
        #         get_body_states_srv = rospy.ServiceProxy(
        #         self.env_prefix + self.simulator_prefix + "/get_body_states", GetBodyStates)
        #         get_body_states_srv.wait_for_service(3)
        #         body_states = get_body_states_srv(request)
        #     except rospy.ServiceException as e:
        #         raise MujocoROSError("Service call failed: {}".format(e))
        #     # index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        #     # idx = index_map[eef_body_name]
        #     idx = body_states.name.index(eef_body_name)
        #     eef_pose_ori = body_states.pose[idx].orientation
        request = GetBodyStatesRequest()
        try:
            get_body_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_body_states", GetBodyStates)
            get_body_states_srv.wait_for_service(3)
            body_states = get_body_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))
        # index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        # idx = index_map[eef_body_name]
        idx = body_states.name.index(eef_body_name)
        eef_pose_ori = body_states.pose[idx].orientation

        if format == "xyzw":
            eef_quat = [-eef_pose_ori.x, -eef_pose_ori.y, -eef_pose_ori.z, -eef_pose_ori.w]
        elif format == "wxyz":
            eef_quat = [-eef_pose_ori.w, -eef_pose_ori.x, -eef_pose_ori.y, -eef_pose_ori.z]
        else:
            raise MujocoROSError("Invalid end effector quaternion format!")

        return eef_quat

    # def get_eef_vel(self):
    #     joint_values = self.get_joint_pos(self._arm_joint_names)
    #     jac_mat = self._moveit_manipulator_group.get_jacobian_matrix(joint_values)
    #     # index_map = dict((name, idx) for idx, name in enumerate(self._moveit_manipulator_group.get_active_joints()))
    #     # index = [index_map[name] for name in self._arm_joint_names]
    #     indices = [self._moveit_manipulator_group.get_active_joints().index(name) for name in self._arm_joint_names]
    #     jac_mat = jac_mat[:, indices]  # switch columns
    #
    #     joint_velocities = self.get_joint_vel(self._arm_joint_names)
    #     eef_vel = np.matmul(jac_mat, joint_velocities)
    #
    #     return eef_vel

    def get_object_pos(self, object_name):
        # TODO (chongyi zheng): object pose estimation using computer vision algorithm
        # body_states_msg = None
        # try:
        #     body_states_msg = rospy.wait_for_message(self.env_prefix + self.simulator_prefix + '/body_states',
        #                                              mujoco_ros_msgs.msg.BodyStates, 3)
        # except rospy.ROSException as e:
        #     MujocoROSError("Message read failed: {}".format(e))
        # body_states_msg = rospy.wait_for_message(
        # self.env_prefix + self.simulator_prefix + '/body_states', mujoco_ros_msgs.msg.BodyStates, 5)
        # with self._body_states_msg_lock:
        #     body_states_msg = copy.deepcopy(self._body_states_msg)
        try:
            request = GetBodyStatesRequest()
            get_body_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_body_states", GetBodyStates)
            get_body_states_srv.wait_for_service(3)
            body_states = get_body_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))
        # index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        # idx = index_map[object_name]
        idx = body_states.name.index(object_name)
        object_pos = [body_states.pose[idx].position.x, body_states.pose[idx].position.y,
                      body_states.pose[idx].position.z]

        return object_pos

    def get_object_quat(self, object_name, format="xyzw"):
        # TODO (chongyi zheng): object pose estimation using computer vision algorithm
        # body_states_msg = None
        # try:
        #     body_states_msg = rospy.wait_for_message(self.env_prefix + self.simulator_prefix + '/body_states',
        #                                              mujoco_ros_msgs.msg.BodyStates, 3)
        # except rospy.ROSException as e:
        #     MujocoROSError("Message read failed: {}".format(e))
        # body_states_msg = rospy.wait_for_message(
        # self.env_prefix + self.simulator_prefix + '/body_states', mujoco_ros_msgs.msg.BodyStates, 5)
        # with self._body_states_msg_lock:
        #     body_states_msg = copy.deepcopy(self._body_states_msg)
        request = GetBodyStatesRequest()
        try:
            get_body_states_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_body_states", GetBodyStates)
            get_body_states_srv.wait_for_service(3)
            body_states = get_body_states_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))
        # index_map = dict((name, idx) for idx, name in enumerate(body_states_msg.name))
        # idx = index_map[object_name]
        idx = body_states.name.index(object_name)
        object_ori = body_states.pose[idx].orientation

        if format == "xyzw":
            object_quat = [object_ori.x, object_ori.y, object_ori.z, object_ori.w]
        elif format == "wxyz":
            object_quat = [object_ori.w, object_ori.x, object_ori.y, object_ori.z]
        else:
            raise MujocoROSError("Invalid object quaternion format!")

        return object_quat

    def get_contacts(self):
        request = GetContactsRequest()
        try:
            get_contacts_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/get_contacts", GetContacts)
            get_contacts_srv.wait_for_service(3)
            contact_geoms = get_contacts_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call falied: {}".format(e))

        return np.array(contact_geoms.geom1), np.array(contact_geoms.geom2)

    def set_fixed_camera(self, camera_id):
        request = SetFixedCameraRequest(camera_id=camera_id)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/set_fixed_camera", SetFixedCamera)
            set_joint_qpos_srv.wait_for_service(3)
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
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/set_joint_qpos", SetJointQPos)
            set_joint_qpos_srv.wait_for_service(3)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_ctrl(self, names, ctrls):
        request = SetCtrlRequest(name=names, ctrl=ctrls)
        try:
            set_ctrl_srv = rospy.ServiceProxy(self.env_prefix + self.simulator_prefix + "/set_ctrl", SetCtrl)
            set_ctrl_srv.wait_for_service(3)
            set_ctrl_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def set_vopt_geomgroup(self, index, value):
        request = SetOptGeomGroupRequest(index=index, value=value)
        try:
            set_joint_qpos_srv = rospy.ServiceProxy(
                self.env_prefix + self.simulator_prefix + "/set_vopt_geomgroup", SetOptGeomGroup)
            set_joint_qpos_srv.wait_for_service(3)
            set_joint_qpos_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

    def reset(self):
        # switch_controller_req = SwitchControllerRequest()
        # switch_controller_req.stop_controllers = ['joint_state_controller']
        # switch_controller_req.start_controllers = []
        # switch_controller_req.strictness = switch_controller_req.STRICT
        # switch_controller_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        # switch_controller_srv.wait_for_service(3)
        # try:
        #     switch_controller_srv(switch_controller_req)
        # except rospy.ServiceException as e:
        #     raise MujocoROSError("Service call failed: {}".format(e))

        request = TriggerRequest()
        try:
            reset_srv = rospy.ServiceProxy(self.env_prefix + self.simulator_prefix + "/reset", Trigger)
            reset_srv.wait_for_service(3)
            reset_srv(request)
        except rospy.ServiceException as e:
            raise MujocoROSError("Service call failed: {}".format(e))

        # switch_controller_req.stop_controllers = []
        # switch_controller_req.start_controllers = ['joint_state_controller']
        # try:
        #     switch_controller_srv(switch_controller_req)
        # except rospy.ServiceException as e:
        #     raise MujocoROSError("Service call failed: {}".format(e))

    # def is_position_valid(self, joint_positions):
    #     assert isinstance(joint_positions, dict) or isinstance(joint_positions, list) or \
    #            isinstance(joint_positions, np.ndarray), "Invalid joint positions to check!"
    #
    #     rs = moveit_msgs.msg.RobotState()
    #     if isinstance(joint_positions, dict):
    #         rs.joint_state.name, rs.joint_state.position = list(joint_positions.keys()),
    #         list(joint_positions.values())
    #     else:
    #         rs.joint_state.name, rs.joint_state.position = self._arm_joint_names, list(joint_positions)
    #     result = self._state_validity.get_state_validity(rs, self.manipulator_group_name)
    #
    #     is_valid = result.valid
    #     return is_valid

    # def goto_arm_positions(self, arm_joint_positions, wait=True):
    #     # if self.is_position_valid(arm_joint_positions):
    #     #     # go to target joint positions
    #     #     self._moveit_manipulator_group.go(arm_joint_positions, wait=wait)
    #     #
    #     #     # calling `stop()` ensures that there is no residual movement
    #     #     # self._moveit_manipulator_group.stop()
    #     # else:
    #     #     raise MujocoROSError("Invalid joint positions: {}".format(arm_joint_positions))
    #     self._moveit_manipulator_group.go(arm_joint_positions, wait=wait)

    # def goto_eef_pose(self, eef_pos, eef_quat, quat_format="xyzw"):
    #     assert np.shape(eef_pos) == (3,) and np.shape(eef_quat) == (4,), "Invalid end effector pose!"
    #
    #     pose = geometry_msgs.msg.PoseStamped()
    #     pose.header.frame_id = self._robot_base_link
    #     pose.pose.position.x = eef_pos[0]
    #     pose.pose.position.y = eef_pos[1]
    #     pose.pose.position.z = eef_pos[2]
    #
    #     if quat_format == "xyzw":
    #         pose.pose.orientation.x = eef_quat[0]
    #         pose.pose.orientation.y = eef_quat[1]
    #         pose.pose.orientation.z = eef_quat[2]
    #         pose.pose.orientation.w = eef_quat[3]
    #     elif quat_format == "wxyz":
    #         pose.pose.orientation.w = eef_quat[0]
    #         pose.pose.orientation.x = eef_quat[1]
    #         pose.pose.orientation.y = eef_quat[2]
    #         pose.pose.orientation.z = eef_quat[3]
    #
    #     # TODO (chongyi zheng): check validity of pose
    #     # go to target pose
    #     # self._moveit_manipulator_group.set_pose_target(pose)
    #     # self._moveit_manipulator_group.go(wait=wait)
    #     # print(rospy.Time.now().to_sec())  # not work
    #     self._cartesian_target_pub.publish(pose)
    #
    #     # calling `stop()` ensures that there is no residual movement
    #     # self._moveit_manipulator_group.stop()
    #
    #     # it is always good to clear your targets after planning with poses.
    #     # self._moveit_manipulator_group.clear_pose_targets()

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

    def jog_eef_pose(self, linear_delta, angular_delta, avoid_collisions=True, linear_delta_scale=1.0,  # 0.125
                     angular_delta_scale=1.0):  # 0.1
        """Send jog message directly"""
        scaled_linear_delta = linear_delta_scale * np.array(linear_delta)
        # to be compatible with MuJoCo rotation direction
        scaled_angular_delta = -1.0 * angular_delta_scale * np.array(angular_delta)

        # if (rospy.Time.now() - self._jog_frame_msg.header.stamp).to_sec() > 0.1:
        jog_frame_msg = jog_msgs.msg.JogFrame()
        # jog_frame_msg.header.stamp = rospy.Time.now()
        jog_frame_msg.header.frame_id = self._jog_base_link
        jog_frame_msg.group_name = self._jog_group
        jog_frame_msg.link_name = self._jog_target_link
        jog_frame_msg.linear_delta.x = scaled_linear_delta[0]
        jog_frame_msg.linear_delta.y = scaled_linear_delta[1]
        jog_frame_msg.linear_delta.z = scaled_linear_delta[2]
        jog_frame_msg.angular_delta.x = scaled_angular_delta[0]
        jog_frame_msg.angular_delta.y = scaled_angular_delta[1]
        jog_frame_msg.angular_delta.z = scaled_angular_delta[2]
        jog_frame_msg.avoid_collisions = avoid_collisions

        # Publish only if the all command are not equal zero
        # Not good, we need to compare slider value by some way...
        if jog_frame_msg.linear_delta.x != 0 or jog_frame_msg.linear_delta.y != 0 or \
            jog_frame_msg.linear_delta.z != 0 or jog_frame_msg.angular_delta.x != 0 or \
            jog_frame_msg.angular_delta.y != 0 or jog_frame_msg.angular_delta.z != 0:
            self._jog_frame_pub.publish(jog_frame_msg)

    def goto_arm_positions(self, arm_joint_positions, time_from_start=5.0, wait=False):
        switch_controller_req = None
        switch_controller_srv = None

        for controller in self._controllers:
            if 'manipulator' in controller['name']:
                # switch controller when necessary
                if len(self._cartesian_controller_names) != 0:
                    try:
                        switch_controller_req = SwitchControllerRequest()
                        switch_controller_req.start_controllers = self._manipulator_controller_names
                        switch_controller_req.stop_controllers = self._cartesian_controller_names
                        switch_controller_req.strictness = switch_controller_req.BEST_EFFORT
                        switch_controller_srv = rospy.ServiceProxy(
                            self.env_prefix + "/controller_manager/switch_controller", SwitchController)
                        switch_controller_srv.wait_for_service(3)
                        switch_controller_srv(switch_controller_req)
                    except rospy.ServiceException as e:
                        raise MujocoROSError("Service call failed: {}".format(e))

                    try:
                        controller['traj_client'].wait_for_server(rospy.Duration(10))
                        rospy.loginfo("{} is ready.".format(self.env_prefix + '/' + controller['name']))
                    except:
                        MujocoROSError("{} is not found on ROS parameter server!".format(
                            self.env_prefix + '/' + controller['name'] + '/follow_joint_trajectory'))

                point = trajectory_msgs.msg.JointTrajectoryPoint()
                point.positions = list(arm_joint_positions.values())
                point.velocities = []
                point.accelerations = []
                point.time_from_start = rospy.Duration().from_sec(time_from_start)

                goal = control_msgs.msg.FollowJointTrajectoryGoal()
                # goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration().from_sec(time_from_start)
                goal.trajectory.joint_names = list(arm_joint_positions.keys())
                goal.trajectory.points.append(point)

                if wait:
                    try:
                        controller["traj_client"].send_goal_and_wait(
                            goal, execute_timeout=rospy.Duration.from_sec(time_from_start),
                            preempt_timeout=rospy.Duration.from_sec(time_from_start))
                    except:
                        raise MujocoROSError("Calling trajectory client failed!")
                else:
                    controller["traj_client"].send_goal(goal)

                # switch back controller when necessary
                if len(self._cartesian_controller_names) != 0:
                    try:
                        switch_controller_req.start_controllers = self._cartesian_controller_names
                        switch_controller_req.stop_controllers = self._manipulator_controller_names
                        switch_controller_req.strictness = switch_controller_req.BEST_EFFORT
                        switch_controller_srv(switch_controller_req)
                    except rospy.ServiceException as e:
                        raise MujocoROSError("Service call failed: {}".format(e))

    def goto_gripper_positions(self, gripper_joint_positions, time_from_start=0.5):
        # Publish trajectory message for gripper controller
        for controller in self._controllers:
            if 'gripper' in controller['name']:
                point = trajectory_msgs.msg.JointTrajectoryPoint()
                point.positions = list(gripper_joint_positions.values())
                point.velocities = []
                point.accelerations = []
                point.time_from_start = rospy.Duration().from_sec(time_from_start)

                traj = trajectory_msgs.msg.JointTrajectory()
                traj.joint_names = list(gripper_joint_positions.keys())
                traj.points.append(point)

                controller["traj_pub"].publish(traj)

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
