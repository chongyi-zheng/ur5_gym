import argparse
import numpy as np
import robosuite as suite
from robosuite.controllers import load_controller_config


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="Lift")
    parser.add_argument("--robots", nargs="+", type=str, default="Panda", help="Which robot(s) to use in the env")
    parser.add_argument("--grippers", nargs="+", type=str, default="PandaGripper",
                        help="Which gripper(s) to use in the env")
    parser.add_argument("--controller", type=str, default="OSC_POSE",
                        help="Choice of controller. Can be 'IK_POSE' or 'OSC_POSE' or 'OSC_POSITION', or "
                             "'JOINT_POSITION' or 'JOINT_VELOCITY' or 'JOINT_TORQUE'")
    parser.add_argument("--camera_view", type=str, default="front_view",
                        help="Camera view. Can be 'frontview', 'birdview', 'agentview', 'sideview', "
                             "'robot0_robotview' or 'robot0_eye_in_hand'")
    args = parser.parse_args()

    # Get controller config
    controller_config = load_controller_config(default_controller=args.controller)

    # Create argument configuration
    config = {
        "env_name": args.environment,
        "robots": args.robots,
        "gripper_types": args.grippers,
        "controller_configs": controller_config,
        "render_camera": args.camera_view,
    }

    # initialize the task
    env = suite.make(
        **config,
        has_renderer=True,
        has_offscreen_renderer=False,
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
    )
    env.reset()

    # Get action limits
    low, high = env.action_spec

    # do visualization
    for i in range(10000):
        action = np.random.uniform(low, high)
        obs, reward, done, _ = env.step(action)
        env.render()
