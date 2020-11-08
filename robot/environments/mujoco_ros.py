import rospy


class MujocoROSError(Exception):
    """Base class for exceptions in MujocoROS class."""
    pass


class MujocoROS:
    def __init__(self, node_name='mujoco_ros', param_prefix='/mujoco_ros'):
        self.node_name = node_name
        self.param_prefix = param_prefix

        rospy.init_node(self.node_name, anonymous=True)

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
        param_name = self.param_prefix + '/timestep'
        mujoco_timestep = self._get_param(param_name)

        return mujoco_timestep

    def joint_pos_indexes(self, robot_joints):
        param_name = self.param_prefix + '/joint_pos_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def joint_vel_indexes(self, robot_joints):
        param_name = self.param_prefix + '/joint_vel_indexes'
        selected_indexes = self._get_param(param_name, selections=robot_joints)

        return selected_indexes

    def actuator_name2id(self, actuator_names):
        param_name = self.param_prefix + '/actuator_name2id'
        selected_indexes = self._get_param(param_name, selections=actuator_names)

        return selected_indexes

    def joint_name2id(self, joint_names):
        param_name = self.param_prefix + '/joint_name2id'
        selected_indexes = self._get_param(param_name, selections=joint_names)

        return selected_indexes

    def body_name2id(self, body_names):
        param_name = self.param_prefix + '/body_name2id'
        if isinstance(body_names, str):
            body_names = [body_names]

        selected_indexes = self._get_param(param_name, selections=body_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def geom_name2id(self, geom_names):
        param_name = self.param_prefix + '/geom_name2id'
        if isinstance(geom_names, str):
            geom_names = [geom_names]

        selected_indexes = self._get_param(param_name, selections=geom_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes

    def geom_id2name(self, geom_ids):
        param_name = self.param_prefix + '/geom_ids'
        if isinstance(geom_ids, str):
            geom_ids = [geom_ids]

        selected_names = self._get_param(param_name, selections=geom_ids)

        if len(selected_names) == 1:
            return selected_names[0]
        else:
            return selected_names

    def site_name2id(self, site_names):
        param_name = self.param_prefix + '/site_name2id'
        if isinstance(site_names, str):
            site_names = [site_names]

        selected_indexes = self._get_param(param_name, selections=site_names)

        if len(selected_indexes) == 1:
            return selected_indexes[0]
        else:
            return selected_indexes
