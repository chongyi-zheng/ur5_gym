from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from src.envs.ur5 import ReacherEnv


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
    reacher_env = ReacherEnv(INITIAL_GOAL, INITIAL_ROBOT_JOINT_POS)

    obs = reacher_env.reset()
    print(obs)

    obs, reward, done, info = reacher_env.step(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    print(obs)
    print(reward)
    print(done)
    print(info)


if __name__ == '__main__':
    main()
