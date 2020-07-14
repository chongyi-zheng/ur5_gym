"""

World base class

Based on https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/worlds/world.py

"""


import os.path as osp


class World(object):
    MODEL_DIR = osp.join(osp.dirname(__file__), 'models')

    def reset(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def get_observation(self):
        raise NotImplementedError

    @property
    def observation_space(self):
        raise NotImplementedError

    def _initialize_world(self):
        raise NotImplementedError
