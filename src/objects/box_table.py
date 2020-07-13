

from src.objects.base import Object


class BoxTable(Object):
    """BoxTable object"""
    def __init__(self, frame='world', name='table'):
        Object.__init__(self, [0.6, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0], frame=frame, name=name)
        self.size = [0.9, 0.9, 0.85]  # length in x, y, and z directions
