from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from src.robots.ur5 import UR5


INITIAL_GOAL = np.array([0.2, 0.0, 0.3])

# up
INITIAL_ROBOT_JOINT_POS = {
    'shoulder_pan_joint': 0.0,
    'shoulder_lift_joint': -1.5707,
    'elbow_joint': 0.0,
    'wrist_1_joint': -1.5707,
    'wrist_2_joint': 0.0,
    'wrist_3_joint': 0.0
}


def main():
    ur5 = UR5(INITIAL_ROBOT_JOINT_POS, 'manipulator')

    ur5.reset()

    obs = ur5.get_observation()
    observation_space = ur5.observation_space
    action_space = ur5.action_space
    print("Done")


if __name__ == '__main__':
    main()
