import copy

from src.objects.base import Object


class Block(Object):
    """Block object"""
    def __init__(self, init_pos, random_delta_range, init_ori=None, frame='world', name='block'):
        """Task Object interface

        Arguments
        ----------
        - init_pos: np.ndarray
            Block's original position

        - random_delta_range: [float, ...]
            Positive, the range that would be used in sampling object' new start position for every episode
            Set it as 0, if you want to keep the block's initial_pos for every episode.

        - init_ori: np.ndarray or None (default = None)
            Block's original orientation

        - frame: str (default = 'world')
            Reference frame of the object pose

        - name: str (default = 'block)
            Name of the block

        Returns
        ----------

        """
        if init_ori is None:
            init_ori = [0.0, 0.0, 0.0, 1.0]
        Object.__init__(self, init_pos, init_ori, frame, name)

        self.random_delta_range = random_delta_range

        self._position = copy.deepcopy(init_pos)
        self._orientation = copy.deepcopy(init_ori)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
