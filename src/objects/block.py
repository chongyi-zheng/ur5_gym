import copy

from src.objects.base import Object


class Block(Object):
    """Block object"""
    def __init__(self, init_pos, random_delta_range, frame='world', name='block'):
        """Task Object interface

        Arguments
        ----------
        - initial_pos: np.ndarray
            Block's original position

        - frame: str (default = 'world')
            Reference frame of the object pose

        - name: str (default = 'block)
            Name of the block

        - random_delta_range: [float, float, float]
            Positive, the range that would be used in sampling object' new start position for every episode
            Set it as 0, if you want to keep the block's initial_pos for every episode.

        Returns
        ----------

        """
        Object.__init__(self, init_pos, [0.0, 0.0, 0.0, 1.0], frame, name)

        self.random_delta_range = random_delta_range

        self._position = copy.deepcopy(init_pos)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
