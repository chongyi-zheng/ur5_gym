"""

World without any objects

Based on https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/worlds/block_world.py

"""

import collections
import os.path as osp

# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, \
    TransformStamped
from mujoco_ros_msgs.msg import ModelStates
import gym
import numpy as np
import rospy

from src.objects import Block, BoxTable
from src.worlds.world import World

# from sawyer.ros.worlds.gazebo import Gazebo
# from sawyer.ros.worlds.world import World
# import sawyer.garage.misc.logger as logger
# try:
#     from sawyer.garage.config import VICON_TOPICS
# except ImportError:
#     raise NotImplementedError(
#         "Please set VICON_TOPICS in sawyer.garage.config_personal.py! "
#         "example 1:"
#         "   VICON_TOPICS = ['<vicon_topic_name>']"
#         "example 2:"
#         "   # if you are not using real robot and vicon system"
#         "   VICON_TOPICS = []")

BLOCK_INIT_POS = [0.6, 0.0, 0.9]  # from XML


class BlockWorld(World):
    """Block world"""
    def __init__(self, moveit_scene, frame_id, simulated=False, resource='/mujoco/model_states'):
        """Initialize a BlockWorld object

        Arguments
        ----------
        - moveit_scene: moveit_commander.PlanningSceneInterface
            Use this to add/Move/Remove objects in MoveIt!

        - frame_id: string
            Use this to add/Move/Remove objects in MoveIt!, reference frame

        - simulated: bool (default = True)
            If simulated

        - resource: str or None (default = None)
            ROS topic name for simulated and real block state

        Returns
        ----------

        """
        self.moveit_scene = moveit_scene
        self.frame_id = frame_id
        self.simulated = simulated
        self.resource = resource

        self._box_table = None
        self._block = None
        self._model_states_sub = None
        # self._blocks = []
        # self._block_states_subs = []

        self._initialize_world()

    def _initialize_world(self):
        """Initialize the block world

        Arguments
        ----------

        Returns
        ----------

        """
        if self.simulated:
            # Gazebo.load_gazebo_model(
            #     'table',
            #     Pose(position=Point(x=0.75, y=0.0, z=0.0)),
            #     osp.join(World.MODEL_DIR, 'cafe_table/model.sdf'))
            # Gazebo.load_gazebo_model(
            #     'block',
            #     Pose(position=Point(x=0.5725, y=0.1265, z=0.90)),
            #     osp.join(World.MODEL_DIR, 'block/model.urdf'))
            self._block = Block(name='block', init_pos=BLOCK_INIT_POS, random_delta_range=[0.15, 0.15])
            # Waiting models to be loaded
            # rospy.sleep(1)
            # self._block_states_subs.append(
            #     rospy.Subscriber('/gazebo/model_states', ModelStates,
            #                      self._update_block_states))
            # self._blocks.append(block)
            self._model_states_sub = rospy.Subscriber(self.resource, ModelStates, self._update_block_states)
        else:
            raise NotImplementedError("Only simulated BlockWorld is available now!")

        self._observation_space = gym.spaces.Box(-np.inf, np.inf, shape=self.get_observation().obs.shape,
                                                 dtype=np.float32)

        # add table to moveit
        self._box_table = BoxTable(self.frame_id)
        self.moveit_scene.add_box(self._box_table.name, self._box_table.init_pose, self._box_table.size)

    def _update_block_states(self, model_states):
        """Read block state from simulated or real robot

        Arguments
        ----------

        Returns
        ----------

        """
        model_names = model_states.name
        block_idx = model_names.index(self._block.name)
        block_pose = model_states.pose[block_idx]
        self._block.position = block_pose.position
        self._block.orientation = block_pose.orientation

        # for block in self._blocks:
        #     block_idx = model_names.index(block.name)
        #     block_pose = model_states.pose[block_idx]
        #     block.position = block_pose.position
        #     block.orientation = block_pose.orientation

    # def _vicon_update_block_states(self, data):
    #     translation = data.transform.translation
    #     rotation = data.transform.rotation
    #     child_frame_id = data.child_frame_id
    #
    #     for block in self._blocks:
    #         if block.resource == child_frame_id:
    #             block.position = translation
    #             block.orientation = rotation

    def _reset_sim(self):
        """Reset the simulation

        Arguments
        ----------

        Returns
        ----------

        """
        # randomize start position of blocks
        block_random_delta = np.zeros(2)
        while np.linalg.norm(block_random_delta) < 0.1:
            block_random_delta = np.random.uniform(-self._block.random_delta_range[:2],
                                                   self._block.random_delta_range[:2], size=2)
        block_pose = Pose(
            position=Point(
                x=self._block.init_pos[0] + block_random_delta[0],
                y=self._block.init_pos[1] + block_random_delta[1],
                z=self._block.init_pos[2])
        )

        # for block in self._blocks:
        #     block_random_delta = np.zeros(2)
        #     while np.linalg.norm(block_random_delta) < 0.1:
        #         block_random_delta = np.random.uniform(
        #             -block.random_delta_range,
        #             block.random_delta_range,
        #             size=2)
        #     Gazebo.set_model_pose(
        #         block.name,
        #         new_pose=Pose(
        #             position=Point(
        #                 x=block.initial_pos.x + block_random_delta[0],
        #                 y=block.initial_pos.y + block_random_delta[1],
        #                 z=block.initial_pos.z)))

    @property
    def observation_space(self):
        """Get observation space of the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        return self._observation_space

    def reset(self):
        """Reset the block world

        Arguments
        ----------

        Returns
        ----------

        """
        if self.simulated:
            self._reset_sim()
        else:
            raise NotImplementedError("Only simulated BlockWorld is available now!")

    def close(self):
        """Terminate the empty world

        Arguments
        ----------

        Returns
        ----------

        """
        self._model_states_sub.unregister()

        if self.simulated:
            # for block in self._blocks:
            #     Gazebo.delete_gazebo_model(block.name)
            # Gazebo.delete_gazebo_model('table')
            pass
        else:
            ready = False
            while not ready:
                ans = input('Are you ready to exit?[Yes/No]\n')
                if ans.lower() == 'yes' or ans.lower() == 'y':
                    ready = True

        self.moveit_scene.remove_world_object(self._box_table.name)

    def get_observation(self):
        """Get the observation from empty world

        Arguments
        ----------

        Returns
        ----------
        - obs: {observation: }

        """
        block_pos = np.array([self._block.position.x, self._block.position.y, self._block.position.z])
        block_ori = np.array([self._block.orientation.x, self._block.orientation.y, self._block.orientation.z,
                              self._block.orientation.w])

        achieved_goal = np.squeeze(block_pos)
        obs = np.concatenate([block_pos, block_ori])

        Observation = collections.namedtuple('Observation',
                                             'obs achieved_goal')

        observation = Observation(obs=obs, achieved_goal=achieved_goal)

        return observation

    # def add_block(self, block):
    #     if self._simulated:
    #         Gazebo.load_gazebo_model(
    #             block.name, Pose(position=block.initial_pos), block.resource)
    #         # Waiting model to be loaded
    #         rospy.sleep(1)
    #     else:
    #         self._block_states_subs.append(
    #             rospy.Subscriber(block.resource, TransformStamped,
    #                              self._vicon_update_block_states))
    #     self._blocks.append(block)
