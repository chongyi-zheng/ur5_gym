"""
Interfaces to MoveIt! kinematics services.

Take from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/robots/kinematics_interfaces.py
"""

from geometry_msgs.msg import PoseStamped
import moveit_msgs.srv
import rospy
from sensor_msgs.msg import JointState


class ForwardKinematics:
    """Interface to MoveIt! forward kinematics service."""

    def __init__(self, srv_name='/compute_fk', js_topic_name='/robot/joint_states'):
        """Interface to MoveIt! forward kinematics service.

        Arguments
        ----------
        - srv_name: str
            The name of the MoveIt! forward kinematics service

        - js_topic_name: str
            The name of the joint state topic

        Returns
        ----------

        """
        self.srv_name = srv_name
        self.js_topic_name = js_topic_name

        self._srv = rospy.ServiceProxy(self.srv_name, moveit_msgs.srv.GetPositionFK)
        self._srv.wait_for_service()

    def terminate(self):
        """Terminate the service proxy.

        Arguments
        ----------

        Returns
        ----------

        """
        self._srv.close()

    def get_fk(self, fk_link_names, joint_names, positions, frame_id='base_link'):
        """Get the forward kinematics of a joint configuration.

        Arguments
        ----------
        - fk_link_names: [str, str, ...]
            List of links that we want to get the forward kinematics from

        - joint_names: [str, str, ...]
            With the joint names to set a position to ask for the FK

        - positions: [float, float, ...]
            Positions of joints

        - frame_id: str
            The reference frame to be used

        Returns
        ----------
        - fk_result: moveit.srv.GetPositionFKResponse
            The result of the forward kinematics service

        """
        fk_request = moveit_msgs.srv.GetPositionFKRequest()
        fk_request.fk_link_names = fk_link_names
        fk_request.robot_state.joint_state = joint_names
        fk_request.robot_state.joint_state.position = positions
        fk_request.header.frame_id = frame_id

        fk_result = self._srv.call(fk_request)
        return fk_result

    def get_current_fk(self, fk_link_names, frame_id='base_link'):
        """Get the current forward kinematics of a set of links.

        Arguments
        ----------
        - fk_link_names: [str, str, ...]
            List of links that we want to get the forward kinematics from

        - frame_id: str
            The reference frame to be used

        Returns
        ----------
        - fk_result: moveit_msgs.srv.GetPositionFKResponse
            The result of the forward kinematics service

        """
        # Subscribe to a joint_states
        js = rospy.wait_for_message(self.js_topic_name, JointState)

        # Call FK service
        fk_result = self.get_fk(fk_link_names, js.name, js.position, frame_id)

        return fk_result


class InverseKinematics:
    """Interface to MoveIt! inverse kinematics service."""

    def __init__(self, srv_name='/compute_ik'):
        """Interface to MoveIt! inverse kinematics service.

        Arguments
        ----------
        - srv_name: str
            The name of the MoveIt! inverse kinematics service

        Returns
        ----------

        """
        self.srv_name = srv_name

        self._srv = rospy.ServiceProxy(self.srv_name, moveit_msgs.srv.GetPositionIK)
        self._srv.wait_for_service()

    def terminate(self):
        """Terminate the service proxy.

        Arguments
        ----------

        Returns
        ----------

        """
        self._srv.close()

    def get_ik(self, group_name, ik_link_name, pose_stamped, avoid_collisions=True, attempts=None, robot_state=None,
               constraints=None):
        """Get the inverse kinematics with a link in a pose in 3d world.

        Arguments
        ----------
        - group_name: str
            Group name, i.e. 'right_arm'

        - ik_link_name: str
            Link that will be in the pose given to evaluate the IK

        - pose_stamped: PoseStamped
            The pose with frame_id of the link

        - avoid_collisions: Bool
            If we want solutions with collision avoidance

        - attempts: Int
            Number of attempts to get an IK

        - robot_state: RobotState
            The robot state where to start searching IK from (optional, current pose will be used if ignored)

        - constraints: Constraints
            The robot state constraints

        Returns
        ----------
        - ik_result: moveit_msgs.srv.GetPositionIKResponse
            The result of the inverse kinematics service

        """
        assert isinstance(pose_stamped, type(PoseStamped))

        ik_request = moveit_msgs.srv.GetPositionIKRequest()
        ik_request.ik_request.group_name = group_name
        if robot_state:
            ik_request.ik_request.robot_state = robot_state
        ik_request.ik_request.avoid_collisions = avoid_collisions
        ik_request.ik_request.ik_link_name = ik_link_name
        ik_request.ik_request.pose_stamped = pose_stamped
        if attempts:
            ik_request.ik_request.attempts = attempts
        else:
            ik_request.ik_request.attempts = 1
        if constraints:
            ik_request.ik_request.constraints = constraints

        ik_result = self._srv.call(ik_request)
        return ik_result


class StateValidity:
    """Interface to MoveIt! StateValidity service."""
    def __init__(self, srv_name='/check_state_validity'):
        """Interface to MoveIt! StateValidity service.

        Arguments
        ----------
        - srv_name: str
            The name of the MoveIt! state validity service

        Returns
        ----------

        """
        self.srv_name = srv_name

        self._srv = rospy.ServiceProxy(self.srv_name, moveit_msgs.srv.GetStateValidity)
        self._srv.wait_for_service()

    def terminate(self):
        """Terminate the service proxy.

        Arguments
        ----------

        Returns
        ----------

        """
        self._srv.close()

    def get_state_validity(self, robot_state, group_name='arm', constraints=None):
        """Get state validity.

        Arguments
        ----------
        - robot_state: RobotState
            The robot state

        - group_name: str
            Planner group name

        - constraints: Constraints
            The robot state constraints

        Returns
        ----------
        - result: moveit_msgs.srv.GetStateValidityResponse
            The result of the state validity service

        """
        sv_request = moveit_msgs.srv.GetStateValidityRequest()
        sv_request.robot_state = robot_state
        sv_request.group_name = group_name
        if constraints:
            sv_request.constraints = constraints
        result = self._srv.call(sv_request)
        return result
