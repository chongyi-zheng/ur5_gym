import rospy
import moveit_msgs.srv


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
