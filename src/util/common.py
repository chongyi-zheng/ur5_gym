"""

Common function utilization

Adapt from https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/util/common.py

"""

from functools import wraps
import threading
import time
import numpy as np


def rate_limited(max_per_second):
    """Rate-limits the decorated function locally, for one process

    Arguments
    ----------
    - max_per_second: int
        Max number of function call in a second

    Returns
    ----------

    """
    lock = threading.Lock()
    min_interval = 1.0 / max_per_second

    def decorate(func):
        decorate.last_time_called = time.time()

        @wraps(func)
        def rate_limited_function(*args, **kwargs):
            lock.acquire()
            try:
                elapsed = time.time() - decorate.last_time_called
                left_to_wait = min_interval - elapsed
                if left_to_wait > 0:
                    time.sleep(left_to_wait)

                return func(*args, **kwargs)
            finally:
                decorate.last_time_called = time.time()
                lock.release()

        return rate_limited_function

    return decorate


def goal_distance(goal_a, goal_b):
    """Compute distance between achieved goal and goal.

    Arguments
    ----------
    - goal_a: np.ndarray
        The first goal

    - goal_b: np.ndarray
        The second goal

    Returns
    ----------
    - distance: float
        Distance between goal_a and goal_b

    """
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)
