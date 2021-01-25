from collections import OrderedDict
# TODO (chongyi zheng): this line has been modified
# from mujoco_py import MjSim, MjRenderContextOffscreen
# from mujoco_py import load_model_from_xml

from robosuite.utils import SimulationError, XMLError
from robot.utils.mujoco_py_render import MujocoROSPyRenderer
from robot.environments.mujoco_ros import MujocoROS

REGISTERED_ENVS = {}


def register_env(target_class):
    REGISTERED_ENVS[target_class.__name__] = target_class


def make(env_name, *args, **kwargs):
    """
    Instantiates a robosuite environment.
    This method attempts to mirror the equivalent functionality of gym.make in a somewhat sloppy way.
    Args:
        env_name (str): Name of the robosuite environment to initialize
        *args: Additional arguments to pass to the specific environment class initializer
        **kwargs: Additional arguments to pass to the specific environment class initializer
    Returns:
        MujocoEnv: Desired robosuite environment
    Raises:
        Exception: [Invalid environment name]
    """
    if env_name not in REGISTERED_ENVS:
        raise Exception(
            "Environment {} not found. Make sure it is a registered environment among: {}".format(
                env_name, ", ".join(REGISTERED_ENVS)
            )
        )
    return REGISTERED_ENVS[env_name](*args, **kwargs)


class EnvMeta(type):
    """Metaclass for registering environments"""

    def __new__(meta, name, bases, class_dict):
        cls = super().__new__(meta, name, bases, class_dict)

        # List all environments that should not be registered here.
        _unregistered_envs = ["MujocoEnv", "RobotEnv"]

        if cls.__name__ not in _unregistered_envs:
            register_env(cls)
        return cls


class MujocoEnv(metaclass=EnvMeta):
    """
    Initializes a Mujoco Environment.
    Args:
        has_renderer (bool): If true, render the simulation state in
            a viewer instead of headless mode.
        has_offscreen_renderer (bool): True if using off-screen rendering.
        render_camera (str): Name of camera to render if `has_renderer` is True. Setting this value to 'None'
            will result in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse
        render_collision_mesh (bool): True if rendering collision meshes
            in camera. False otherwise.
        render_visual_mesh (bool): True if rendering visual meshes
            in camera. False otherwise.
        control_freq (float): how many control signals to receive
            in every simulated second. This sets the amount of simulation time
            that passes between every action input.
        horizon (int): Every episode lasts for exactly @horizon timesteps.
        ignore_done (bool): True if never terminating the environment (ignore @horizon).
        hard_reset (bool): If True, re-loads model, sim, and render object upon a reset call, else,
            only calls sim.reset and resets all robosuite-internal variables
    Raises:
        ValueError: [Invalid renderer selection]
    """

    def __init__(
            self,
            has_renderer=False,
            has_offscreen_renderer=True,
            render_camera="frontview",
            render_collision_mesh=False,
            render_visual_mesh=True,
            control_freq=10,
            horizon=1000,
            ignore_done=False,
            hard_reset=True,
            ros_env_prefix="env",
    ):
        # First, verify that both the on- and off-screen renderers are not being used simultaneously
        if has_renderer is True and has_offscreen_renderer is True:
            raise ValueError("the onscreen and offscreen renderers cannot be used simultaneously.")

        # Rendering-specific attributes
        self.has_renderer = has_renderer
        self.has_offscreen_renderer = has_offscreen_renderer
        self.render_camera = render_camera
        self.render_collision_mesh = render_collision_mesh
        self.render_visual_mesh = render_visual_mesh
        self.viewer = None

        # Simulation-specific attributes
        self.control_freq = control_freq
        self.horizon = horizon
        self.ignore_done = ignore_done
        self.hard_reset = hard_reset
        self.ros_env_prefix = ros_env_prefix
        self.model = None
        self.cur_time = None
        self.model_timestep = None
        self.control_timestep = None
        self.deterministic_reset = False            # Whether to add randomized resetting of objects / robot joints

        # Load the model
        self._load_model()

        # Initialize the simulation
        self._initialize_sim()

        # Run all further internal (re-)initialization required
        self._reset_internal()

    def initialize_time(self, control_freq):
        """
        Initializes the time constants used for simulation.
        Args:
            control_freq (float): Hz rate to run control loop at within the simulation
        """
        self.cur_time = 0
        self.model_timestep = self.sim.timestep
        if self.model_timestep <= 0:
            raise XMLError("xml model defined non-positive time step")
        self.control_freq = control_freq
        if control_freq <= 0:
            raise SimulationError(
                "control frequency {} is invalid".format(control_freq)
            )
        self.control_timestep = 1. / control_freq

    def _load_model(self):
        """Loads an xml model, puts it in self.model"""
        pass

    def _get_reference(self):
        """
        Sets up references to important components. A reference is typically an
        index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        pass

    def _initialize_sim(self, xml_string=None, ros_xml_param_name='mujoco_model_xml'):
        """
        Creates a MjSim object and stores it in self.sim. If @xml_string is specified, the MjSim object will be created
        from the specified xml_string. Else, it will pull from self.model to instantiate the simulation
        Args:
            xml_string (str): If specified, creates MjSim object from this filepath
        """
        # if we have an xml string, use that to create the sim. Otherwise, use the local model
        # self.mjpy_model = load_model_from_xml(xml_string) if xml_string else self.model.get_model(mode="mujoco_py")
        xml_string = xml_string if xml_string else self.model.get_xml()

        # TODO (chongyi zheng): Do we need to load xml_string?
        # if rosgraph.is_master_online():
        #     rospy.set_param(ros_xml_param_name, xml_string)
        # else:
        #     raise SystemError("ROS master is not running!")

        # Create the simulation instance and run a single step to make sure changes have propagated through sim state
        # self.sim = MjSim(self.mjpy_model)
        # self.sim.step()
        if self.ros_env_prefix.startswith('/') or self.ros_env_prefix == "":
            ros_env_prefix = self.ros_env_prefix
        else:
            ros_env_prefix = '/' + self.ros_env_prefix
        self.sim = MujocoROS('ur5e_robotiq_2f_85_mujoco_ros', env_prefix=ros_env_prefix)

        # Setup sim time based on control frequency
        self.initialize_time(self.control_freq)

    def reset(self):
        """
        Resets simulation.
        Returns:
            OrderedDict: Environment observation space after reset occurs
        """
        # TODO(yukez): investigate black screen of death
        # Use hard reset if requested
        if self.hard_reset and not self.deterministic_reset:
            self._destroy_viewer()
            self._load_model()
            self._initialize_sim()
        # Else, we only reset the sim internally
        else:
            # TODO (chongyi zheng): delete this
            # reset_success = False
            # while not reset_success:
            #     self.sim.terminate()
            #     self.
            #     sim.spawn()
            #     reset_success = self.sim.reset_moveit()
            #     # reset_success = self.sim.reset_controllers()
            self.sim.terminate()
            self.sim.spawn()
            # self.sim.reset_moveit()
            # self.sim.reset_controllers()
        # Reset necessary robosuite-centric variables
        self._reset_internal()
        # TODO (ongyi zheng): delete this
        # self.sim.forward()
        return self._get_observation()

    def _reset_internal(self):
        """Resets simulation internal configurations."""

        # TODO (chongyi zheng): delete viewer cause we cannot do this with real robot?
        # create visualization screen or renderer
        if self.has_renderer:
            # self.viewer = MujocoROSPyRenderer(self.sim)
            # TODO (chongyi zheng): set through ros
            # self.viewer.viewer.vopt.geomgroup[0] = (1 if self.render_collision_mesh else 0)
            # self.viewer.viewer.vopt.geomgroup[1] = (1 if self.render_visual_mesh else 0)
            # self.viewer.viewer.set_vopt_geomgroup(0, 1 if self.render_collision_mesh else 0)
            # self.viewer.viewer.set_vopt_geomgroup(1, 1 if self.render_visual_mesh else 0)
            self.sim.set_vopt_geomgroup(0, 1 if self.render_collision_mesh else 0)
            self.sim.set_vopt_geomgroup(1, 1 if self.render_visual_mesh else 0)

            # hiding the overlay speeds up rendering significantly
            # self.viewer.viewer._hide_overlay = True

            # make sure mujoco-py doesn't block rendering frames
            # (see https://github.com/StanfordVL/robosuite/issues/39)
            # self.viewer.viewer._render_every_frame = True

            # Set the camera angle for viewing
            if self.render_camera is not None:
                self.sim.set_fixed_camera(camera_id=self.sim.camera_name2id(self.render_camera))

        elif self.has_offscreen_renderer:
            # TODO (chongyi zheng): Do we need this?
            pass
            # if self.sim._render_context_offscreen is None:
            #     render_context = MjRenderContextOffscreen(self.sim)
            #     self.sim.add_render_context(render_context)
            # self.sim._render_context_offscreen.vopt.geomgroup[0] = (1 if self.render_collision_mesh else 0)
            # self.sim._render_context_offscreen.vopt.geomgroup[1] = (1 if self.render_visual_mesh else 0)

        # Set the camera angle for viewing
        if self.render_camera is not None:
            self.sim.set_fixed_camera(camera_id=self.sim.camera_name2id(self.render_camera))

        # additional housekeeping
        # self.sim_state_initial = self.sim.get_state()  # TODO (chongyi zheng): this line seems useless
        self._get_reference()
        self.cur_time = 0
        self.timestep = 0
        self.done = False

    def _get_observation(self):
        """
        Grabs observations from the environment.
        Returns:
            OrderedDict: OrderedDict containing observations [(name_string, np.array), ...]
        """
        return OrderedDict()

    def step(self, action):
        """
        Takes a step in simulation with control command @action.
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            4-tuple:
                - (OrderedDict) observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        Raises:
            ValueError: [Steps past episode termination]
        """
        if self.done:
            raise ValueError("executing action in terminated episode")

        self.timestep += 1

        # Since the env.step frequency is slower than the mjsim timestep frequency, the internal controller will output
        # multiple torque commands in between new high level action commands. Therefore, we need to denote via
        # 'policy_step' whether the current step we're taking is simply an internal update of the controller,
        # or an actual policy update
        policy_step = True

        # Loop through the simulation at the model timestep rate until we're ready to take the next policy step
        # (as defined by the control frequency specified at the environment level)
        # for i in range(int(self.control_timestep / self.model_timestep)):
        #     # self.sim.forward()
        #     self._pre_action(action, policy_step)
        #     # self.sim.step()
        #     policy_step = False
        self._pre_action(action, policy_step)

        # Note: this is done all at once to avoid floating point inaccuracies
        # self.cur_time += self.control_timestep
        self.cur_time += self.model_timestep

        reward, done, info = self._post_action(action)
        return self._get_observation(), reward, done, info
        # return self._get_observation(), None, None, None
        # return None, None, None, None

    def _pre_action(self, action, policy_step=False):
        """
        Do any preprocessing before taking an action.
        Args:
            action (np.array): Action to execute within the environment
            policy_step (bool): Whether this current loop is an actual policy step or internal sim update step
        """
        self.sim.data.ctrl[:] = action

    def _post_action(self, action):
        """
        Do any housekeeping after taking an action.
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            3-tuple:
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) empty dict to be filled with information by subclassed method
        """
        reward = self.reward(action)

        # done if number of elapsed timesteps is greater than horizon
        self.done = (self.timestep >= self.horizon) and not self.ignore_done
        return reward, self.done, {}

    def reward(self, action):
        """
        Reward should be a function of state and action
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            float: Reward from environment
        """
        raise NotImplementedError

    def render(self):
        """
        Renders to an on-screen window.
        """
        self.viewer.render()

    def observation_spec(self):
        """
        Returns an observation as observation specification.
        An alternative design is to return an OrderedDict where the keys
        are the observation names and the values are the shapes of observations.
        We leave this alternative implementation commented out, as we find the
        current design is easier to use in practice.
        Returns:
            OrderedDict: Observations from the environment
        """
        observation = self._get_observation()
        return observation

    @property
    def action_spec(self):
        """
        Action specification should be implemented in subclasses.
        Action space is represented by a tuple of (low, high), which are two numpy
        vectors that specify the min/max action limits per dimension.
        """
        raise NotImplementedError

    @property
    def action_dim(self):
        """
        Size of the action space
        Returns:
            int: Action space dimension
        """
        raise NotImplementedError

    def reset_from_xml_string(self, xml_string):
        """
        Reloads the environment from an XML description of the environment.
        Args:
            xml_string (str): Filepath to the xml file that will be loaded directly into the sim
        """

        # if there is an active viewer window, destroy it
        self.close()

        # Since we are reloading from an xml_string, we are deterministically resetting
        self.deterministic_reset = True

        # initialize sim from xml
        self._initialize_sim(xml_string=xml_string)

        # Now reset as normal
        self.reset()

        # Turn off deterministic reset
        self.deterministic_reset = False

    def find_contacts(self, geoms_1, geoms_2):
        """
        Finds contact between two geom groups.
        Args:
            geoms_1 (list of str): a list of geom names
            geoms_2 (list of str): another list of geom names
        Returns:
            generator: iterator of all contacts between @geoms_1 and @geoms_2
        """
        for contact in self.sim.data.contact[0 : self.sim.data.ncon]:
            # check contact geom in geoms
            c1_in_g1 = self.sim.model.geom_id2name(contact.geom1) in geoms_1
            c2_in_g2 = self.sim.model.geom_id2name(contact.geom2) in geoms_2
            # check contact geom in geoms (flipped)
            c2_in_g1 = self.sim.model.geom_id2name(contact.geom2) in geoms_1
            c1_in_g2 = self.sim.model.geom_id2name(contact.geom1) in geoms_2
            if (c1_in_g1 and c2_in_g2) or (c1_in_g2 and c2_in_g1):
                yield contact

    def _check_success(self):
        """
        Checks if the task has been completed. Should be implemented by subclasses
        Returns:
            bool: True if the task has been completed
        """
        raise NotImplementedError

    def _destroy_viewer(self):
        """
        Destroys the current mujoco renderer instance if it exists
        """
        # if there is an active viewer window, destroy it
        if self.viewer is not None:
            self.viewer.close()  # change this to viewer.finish()?
            self.viewer = None

    def close(self):
        """Do any cleanup necessary here."""
        self._destroy_viewer()
