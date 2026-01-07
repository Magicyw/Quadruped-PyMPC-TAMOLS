# Description: This script is used to simulate the full model of the robot in mujoco
import pathlib
import threading

# Authors:
# Giulio Turrisi, Daniel Ordonez
import time
from datetime import datetime
from os import PathLike
from pprint import pprint

import mujoco
import numpy as np

# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.mujoco.visual import render_sphere, render_vector
from gym_quadruped.utils.quadruped_utils import LegsAttr
from tqdm import tqdm

# Helper functions for plotting
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco

# PyMPC controller imports
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

# Import for MATLAB file export
try:
    from scipy.io import savemat
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

def keyboard_listener(video_recorder, stop_event):
    """Listen for keyboard input in a separate thread.
    
    This function runs in a daemon thread and listens for the 'V' key to toggle
    video recording. Uses a simple polling approach to avoid blocking Ctrl+C.
    
    Args:
        video_recorder: VideoRecorder instance to control
        stop_event: Threading event to signal when to stop listening
    """
    try:
        import readchar
        print("\n[Keyboard Listener] Press 'V' or 'v' to toggle video recording")
        print("[Keyboard Listener] Press Ctrl+C to exit")

        while not stop_event.is_set():
            try:
                # Use readkey which is less blocking than readchar
                key = readchar.readkey()
                if key and key.lower() == 'v':
                    video_recorder.toggle_recording()
            except KeyboardInterrupt:
                # Re-raise to ensure it propagates
                raise
            except Exception:
                # Handle any read errors (e.g., EOF)
                if stop_event.is_set():
                    break
                time.sleep(0.1)
    except KeyboardInterrupt:
        # Propagate the interrupt to the main thread
        import os
        import signal
        os.kill(os.getpid(), signal.SIGINT)
    except ImportError:
        print("⚠️  readchar not available. Video recording toggle via keyboard disabled.")
        print("   Install with: pip install readchar")


# Standard leg order used throughout simulation
STANDARD_LEG_ORDER = ['FL', 'FR', 'RL', 'RR']


class MatLogger:
    """Logger for recording simulation data and exporting to MATLAB .mat format.
    
    Writes data incrementally to file after each step to avoid memory buildup
    and ensure data is not lost if simulation crashes.
    """
    
    def __init__(self, state_obs_names, quadrupedpympc_observables_names, filepath, write_every_n_steps=1):
        """Initialize the MatLogger.
        
        Args:
            state_obs_names: List of state observation names from QuadrupedEnv
            quadrupedpympc_observables_names: Tuple of controller observable names
            filepath: Path where the .mat file should be saved
            write_every_n_steps: Write to disk every N steps (default: 1 for immediate writing)
        
        Raises:
            ValueError: If filepath is None or write_every_n_steps is not positive
        """
        if filepath is None:
            raise ValueError("filepath cannot be None")
        if not isinstance(write_every_n_steps, int) or write_every_n_steps <= 0:
            raise ValueError(f"write_every_n_steps must be a positive integer, got {write_every_n_steps}")
        
        self.state_obs_names = state_obs_names
        self.quadrupedpympc_observables_names = quadrupedpympc_observables_names
        self.filepath = pathlib.Path(filepath)
        self.write_every_n_steps = write_every_n_steps
        
        # Storage for data (will be written incrementally)
        self.all_data = []
        
        # Track dimensions of each observable (determined dynamically)
        self.obs_dimensions = {}
        self.header = None
        
        # Counter for batch writing
        self.step_count = 0
        
        # Ensure directory exists
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        
    def _flatten_value(self, value):
        """Flatten a value to 1D array, handling None, scalars, arrays, LegsAttr, etc."""
        if value is None:
            return None
        elif isinstance(value, (int, float, np.integer, np.floating)):
            return np.array([value], dtype=np.float64)
        elif isinstance(value, LegsAttr):
            # Flatten LegsAttr by concatenating all leg values in standard order
            flattened = []
            for leg_name in STANDARD_LEG_ORDER:
                leg_val = getattr(value, leg_name, None)
                if leg_val is not None:
                    leg_flat = np.atleast_1d(leg_val).flatten()
                    flattened.append(leg_flat)
            if flattened:
                return np.concatenate(flattened).astype(np.float64)
            else:
                return None
        elif isinstance(value, np.ndarray):
            return value.flatten().astype(np.float64)
        elif isinstance(value, (list, tuple)):
            return np.array(value, dtype=np.float64).flatten()
        else:
            # Try to convert to array
            try:
                return np.array(value, dtype=np.float64).flatten()
            except (TypeError, ValueError):
                return None
    
    def _build_header(self, time, state, ctrl_state):
        """Build the header by inferring dimensions from actual data."""
        header = ['time']
        
        # Build header from state observations
        for obs_name in self.state_obs_names:
            value = state.get(obs_name, None)
            flat_value = self._flatten_value(value)
            
            if flat_value is not None and len(flat_value) > 0:
                dim = len(flat_value)
                self.obs_dimensions[obs_name] = dim
                
                if dim == 1:
                    header.append(obs_name)
                else:
                    for i in range(dim):
                        header.append(f"{obs_name}_{i}")
            else:
                # If None or empty, we'll need to wait for a valid value
                self.obs_dimensions[obs_name] = None
        
        # Build header from controller observations
        for obs_name in self.quadrupedpympc_observables_names:
            value = ctrl_state.get(obs_name, None)
            flat_value = self._flatten_value(value)
            
            if flat_value is not None and len(flat_value) > 0:
                dim = len(flat_value)
                self.obs_dimensions[obs_name] = dim
                
                if dim == 1:
                    header.append(obs_name)
                else:
                    for i in range(dim):
                        header.append(f"{obs_name}_{i}")
            else:
                # If None or empty, we'll need to wait for a valid value
                self.obs_dimensions[obs_name] = None
        
        self.header = header
        return header
    
    def _update_dimensions(self, state, ctrl_state):
        """Update dimensions for observables that were initially None."""
        updated = False
        
        # Update state observation dimensions
        for obs_name in self.state_obs_names:
            if self.obs_dimensions.get(obs_name) is None:
                value = state.get(obs_name, None)
                flat_value = self._flatten_value(value)
                if flat_value is not None and len(flat_value) > 0:
                    self.obs_dimensions[obs_name] = len(flat_value)
                    updated = True
        
        # Update controller observation dimensions
        for obs_name in self.quadrupedpympc_observables_names:
            if self.obs_dimensions.get(obs_name) is None:
                value = ctrl_state.get(obs_name, None)
                flat_value = self._flatten_value(value)
                if flat_value is not None and len(flat_value) > 0:
                    self.obs_dimensions[obs_name] = len(flat_value)
                    updated = True
        
        # Rebuild header if dimensions were updated
        if updated:
            self._rebuild_header()
    
    def _rebuild_header(self):
        """Rebuild header after dimension updates."""
        header = ['time']
        
        for obs_name in self.state_obs_names:
            dim = self.obs_dimensions.get(obs_name, None)
            if dim is not None:
                if dim == 1:
                    header.append(obs_name)
                else:
                    for i in range(dim):
                        header.append(f"{obs_name}_{i}")
        
        for obs_name in self.quadrupedpympc_observables_names:
            dim = self.obs_dimensions.get(obs_name, None)
            if dim is not None:
                if dim == 1:
                    header.append(obs_name)
                else:
                    for i in range(dim):
                        header.append(f"{obs_name}_{i}")
        
        self.header = header
    
    def record_step(self, time, state, ctrl_state):
        """Record data from a single simulation step and write to file.
        
        Args:
            time: Simulation time for this step
            state: State dictionary from env.step()
            ctrl_state: Controller state dictionary from quadrupedpympc_wrapper.get_obs()
        """
        # Initialize header on first call
        if self.header is None:
            self._build_header(time, state, ctrl_state)
        else:
            # Check if we need to update dimensions for previously None values
            self._update_dimensions(state, ctrl_state)
        
        # Build data row
        row = [time]
        
        # Add state observations
        for obs_name in self.state_obs_names:
            value = state.get(obs_name, None)
            flat_value = self._flatten_value(value)
            
            expected_dim = self.obs_dimensions.get(obs_name, 0)
            if expected_dim is None:
                expected_dim = 0
            
            if flat_value is not None and len(flat_value) > 0:
                # Pad or truncate to expected dimension
                if len(flat_value) < expected_dim:
                    flat_value = np.pad(flat_value, (0, expected_dim - len(flat_value)), constant_values=0)
                elif len(flat_value) > expected_dim:
                    flat_value = flat_value[:expected_dim]
                row.extend(flat_value.tolist())
            else:
                # Fill with zeros
                row.extend([0.0] * expected_dim)
        
        # Add controller observations
        for obs_name in self.quadrupedpympc_observables_names:
            value = ctrl_state.get(obs_name, None)
            flat_value = self._flatten_value(value)
            
            expected_dim = self.obs_dimensions.get(obs_name, 0)
            if expected_dim is None:
                expected_dim = 0
            
            if flat_value is not None and len(flat_value) > 0:
                # Pad or truncate to expected dimension
                if len(flat_value) < expected_dim:
                    flat_value = np.pad(flat_value, (0, expected_dim - len(flat_value)), constant_values=0)
                elif len(flat_value) > expected_dim:
                    flat_value = flat_value[:expected_dim]
                row.extend(flat_value.tolist())
            else:
                # Fill with zeros
                row.extend([0.0] * expected_dim)
        
        self.all_data.append(row)
        self.step_count += 1
        
        # Write to file every N steps
        if self.step_count % self.write_every_n_steps == 0:
            self._write_to_file()
    
    def _write_to_file(self):
        """Write accumulated data to the .mat file.
        
        Raises:
            IOError: If file write fails due to disk full, permissions, etc.
        """
        if not SCIPY_AVAILABLE:
            raise ImportError(
                "scipy is required for .mat file export but is not installed. "
                "Please install it with: pip install scipy"
            )
        
        if not self.all_data:
            # No data to write yet - this is normal on first call before any steps
            return
        
        # Convert to numpy array
        data = np.array(self.all_data, dtype=np.float64)
        
        # Ensure header and data columns match - this should never happen if logic is correct
        if self.header is not None and data.shape[1] != len(self.header):
            error_msg = (
                f"Data dimension mismatch: Header has {len(self.header)} columns "
                f"but data has {data.shape[1]} columns. This indicates a bug in data collection."
            )
            raise ValueError(error_msg)
        
        # Create the .mat structure
        mat_dict = {
            'header': np.array(self.header, dtype=object),
            'data': data
        }
        
        # Save to .mat file with error handling
        try:
            savemat(self.filepath, mat_dict)
        except (OSError, IOError) as e:
            raise IOError(
                f"Failed to write .mat file to {self.filepath}: {e}. "
                "Check disk space and file permissions."
            ) from e
    
    def flush(self):
        """Ensure all data is written to file."""
        if self.all_data and self.step_count % self.write_every_n_steps != 0:
            self._write_to_file()
        if self.all_data:
            print(f"✓ Saved {len(self.all_data)} steps to {self.filepath}")



def _format_param_for_filename(param):
    """Format a parameter (scalar or range) as a string for filename.
    
    Args:
        param: Either a scalar value or a tuple/list of (min, max)
    
    Returns:
        Formatted string like "1.0" or "0.5-1.0"
    """
    if isinstance(param, (tuple, list)):
        return f"{param[0]:.1f}-{param[1]:.1f}"
    else:
        return f"{param:.1f}"


def run_simulation(
    qpympc_cfg,
    process=0,
    num_episodes=500,
    num_seconds_per_episode=600,
    ref_base_lin_vel=(0.0, 4.0),
    ref_base_ang_vel=(-0.4, 0.4),
    friction_coeff=(0.5, 1.0),
    base_vel_command_type="human",
    seed=0,
    render=True,
    recording_path: PathLike = None,
    mat_logging: bool = True,
):
    np.set_printoptions(precision=3, suppress=True)
    np.random.seed(seed)

    robot_name = qpympc_cfg.robot
    hip_height = qpympc_cfg.hip_height
    robot_leg_joints = qpympc_cfg.robot_leg_joints
    robot_feet_geom_names = qpympc_cfg.robot_feet_geom_names
    scene_name = qpympc_cfg.simulation_params["scene"]
    simulation_dt = qpympc_cfg.simulation_params["dt"]
    
    # # Load scene configuration (handles stepping stones and custom terrains)
    # stepping_stones_params = qpympc_cfg.simulation_params.get("stepping_stones_params", None)
    # scene_arg, custom_scene_path = load_scene_for_quadruped_env(scene_name, stepping_stones_params)

    # if custom_scene_path:
    #     print(f"Loading custom scene from: {custom_scene_path}")

    # Save all observables available.
    state_obs_names = list(QuadrupedEnv.ALL_OBS)  # + list(IMU.ALL_OBS)

    # Create the quadruped robot environment -----------------------------------------------------------
    # Note: gym_quadruped may need the scene parameter to be a path for custom scenes
    # If it doesn't support custom paths directly, you may need to modify gym_quadruped
    # or copy the XML to gym_quadruped's scene directory
    env = QuadrupedEnv(
        robot=robot_name,
        scene=scene_name,  # This will be either a built-in name or a path
        sim_dt=simulation_dt,
        ref_base_lin_vel=np.asarray(ref_base_lin_vel) * hip_height,  # pass a float for a fixed value
        ref_base_ang_vel=ref_base_ang_vel,  # pass a float for a fixed value
        ground_friction_coeff=friction_coeff,  # pass a float for a fixed value
        base_vel_command_type=base_vel_command_type,  # "forward", "random", "forward+rotate", "human"
        state_obs_names=tuple(state_obs_names),  # Desired quantities in the 'state' vec
    )
    pprint(env.get_hyperparameters())
    env.mjModel.opt.gravity[2] = -qpympc_cfg.gravity_constant

    # Some robots require a change in the zero joint-space configuration. If provided apply it
    if qpympc_cfg.qpos0_js is not None:
        env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], qpympc_cfg.qpos0_js))

    env.reset(random=False)
    if render:
        env.render()  # Pass in the first render call any mujoco.viewer.KeyCallbackType
        env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False

        # Initialize video recorder
        from video_recorder import VideoRecorder
        video_recorder = VideoRecorder(env.viewer, env.mjModel, env.mjData)

        # Start keyboard listener thread for video recording
        keyboard_stop_event = threading.Event()
        keyboard_thread = threading.Thread(
            target=keyboard_listener,
            args=(video_recorder, keyboard_stop_event),
            daemon=True
        )
        keyboard_thread.start()
    else:
        video_recorder = None
        keyboard_stop_event = None
        keyboard_thread = None
    # Initialization of variables used in the main control loop --------------------------------

    # Torque vector
    tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])
    # Torque limits
    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
    )

    # Feet positions and Legs order
    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
    legs_order = ["FL", "FR", "RL", "RR"]

    # Create HeightMap -----------------------------------------------------------------------
    if qpympc_cfg.simulation_params["visual_foothold_adaptation"] != "blind":
        from gym_quadruped.sensors.heightmap import HeightMap

        resolution_heightmap = 0.04
        num_rows_heightmap = 13
        num_cols_heightmap = 7
        heightmaps = LegsAttr(
            FL=HeightMap(num_rows=num_rows_heightmap, num_cols=num_cols_heightmap, 
                         dist_x=resolution_heightmap, dist_y=resolution_heightmap, 
                         mj_model=env.mjModel, mj_data=env.mjData),
            FR=HeightMap(num_rows=num_rows_heightmap, num_cols=num_cols_heightmap, 
                         dist_x=resolution_heightmap, dist_y=resolution_heightmap, 
                         mj_model=env.mjModel, mj_data=env.mjData),
            RL=HeightMap(num_rows=num_rows_heightmap, num_cols=num_cols_heightmap, 
                         dist_x=resolution_heightmap, dist_y=resolution_heightmap, 
                         mj_model=env.mjModel, mj_data=env.mjData),
            RR=HeightMap(num_rows=num_rows_heightmap, num_cols=num_cols_heightmap, 
                         dist_x=resolution_heightmap, dist_y=resolution_heightmap, 
                         mj_model=env.mjModel, mj_data=env.mjData),
        )
    else:
        heightmaps = None

    # Quadruped PyMPC controller initialization -------------------------------------------------------------
    quadrupedpympc_observables_names = (
        "ref_base_height",
        "ref_base_angles",
        "ref_feet_pos",
        "nmpc_GRFs",
        "nmpc_footholds",
        "swing_time",
        "phase_signal",
        "lift_off_positions",
        # "base_lin_vel_err",
        # "base_ang_vel_err",
        # "base_poz_z_err",
    )

    quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=tuple(legs_order),
        feet_geom_id=env._feet_geom_id,
        quadrupedpympc_observables_names=quadrupedpympc_observables_names,
    )

    # Data recording -------------------------------------------------------------------------------------------
    if recording_path is not None:
        from gym_quadruped.utils.data.h5py import H5Writer

        root_path = pathlib.Path(recording_path)
        root_path.mkdir(exist_ok=True)
        dataset_path = (
            root_path
            / f"{robot_name}/{scene_name}"
            / f"lin_vel={ref_base_lin_vel} ang_vel={ref_base_ang_vel} friction={friction_coeff}"
            / f"ep={num_episodes}_steps={int(num_seconds_per_episode // simulation_dt):d}.h5"
        )
        h5py_writer = H5Writer(
            file_path=dataset_path,
            env=env,
            extra_obs=None,  # TODO: Make this automatically configured. Not hardcoded
        )
        print(f"\n Recording data to: {dataset_path.absolute()}")
    else:
        h5py_writer = None
    
    # MATLAB .mat file logging ---------------------------------------------------------------------------------
    mat_filepath = None  # Initialize to None
    if mat_logging:
        if not SCIPY_AVAILABLE:
            print("⚠️  scipy is not installed. MATLAB .mat logging disabled.")
            print("   Install scipy with: pip install scipy")
            mat_logger = None
        else:
            # Create log directory and filename
            log_root = pathlib.Path("log") / scene_name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Format velocity and friction for filename using helper function
            lin_vel_str = _format_param_for_filename(ref_base_lin_vel)
            ang_vel_str = _format_param_for_filename(ref_base_ang_vel)
            friction_str = _format_param_for_filename(friction_coeff)
            
            mat_filename = (
                f"{robot_name}_{scene_name}_"
                f"linvel{lin_vel_str}_angvel{ang_vel_str}_"
                f"friction{friction_str}_"
                f"seed{seed}_{timestamp}.mat"
            )
            mat_filepath = log_root / mat_filename
            
            mat_logger = MatLogger(
                state_obs_names=state_obs_names,
                quadrupedpympc_observables_names=quadrupedpympc_observables_names,
                filepath=mat_filepath,
                write_every_n_steps=1  # Write after each step
            )
            print(f"\n MATLAB .mat logging enabled. Will save to: {mat_filepath.absolute()}")
    else:
        mat_logger = None

    # -----------------------------------------------------------------------------------------------------------
    RENDER_FREQ = 30  # Hz
    N_EPISODES = num_episodes
    N_STEPS_PER_EPISODE = int(num_seconds_per_episode // simulation_dt)
    last_render_time = time.time()

    state_obs_history, ctrl_state_history = [], []
    for episode_num in range(N_EPISODES):
        ep_state_history, ep_ctrl_state_history, ep_time = [], [], []
        for _ in tqdm(range(N_STEPS_PER_EPISODE), desc=f"Ep:{episode_num:d}-steps:", total=N_STEPS_PER_EPISODE):
            # Update value from SE or Simulator ----------------------
            feet_pos = env.feet_pos(frame="world")
            feet_vel = env.feet_vel(frame='world')
            hip_pos = env.hip_positions(frame="world")
            base_lin_vel = env.base_lin_vel(frame="world")
            base_ang_vel = env.base_ang_vel(frame="base")
            base_ori_euler_xyz = env.base_ori_euler_xyz
            base_pos = env.base_pos
            com_pos = env.com

            # Get the reference base velocity in the world frame
            ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()

            # Get the inertia matrix
            if qpympc_cfg.simulation_params["use_inertia_recomputation"]:
                inertia = env.get_base_inertia().flatten()  # Reflected inertia of base at qpos, in world frame
            else:
                inertia = qpympc_cfg.inertia.flatten()

            # Get the qpos and qvel
            qpos, qvel = env.mjData.qpos, env.mjData.qvel
            # Idx of the leg
            legs_qvel_idx = env.legs_qvel_idx  # leg_name: [idx1, idx2, idx3] ...
            legs_qpos_idx = env.legs_qpos_idx  # leg_name: [idx1, idx2, idx3] ...
            joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)

            # Get Centrifugal, Coriolis, Gravity, Friction for the swing controller
            legs_mass_matrix = env.legs_mass_matrix
            legs_qfrc_bias = env.legs_qfrc_bias
            legs_qfrc_passive = env.legs_qfrc_passive

            # Compute feet jacobians
            feet_jac = env.feet_jacobians(frame='world', return_rot_jac=False)
            feet_jac_dot = env.feet_jacobians_dot(frame='world', return_rot_jac=False)

            # Quadruped PyMPC controller --------------------------------------------------------------
            tau = quadrupedpympc_wrapper.compute_actions(
                com_pos,
                base_pos,
                base_lin_vel,
                base_ori_euler_xyz,
                base_ang_vel,
                feet_pos,
                hip_pos,
                joints_pos,
                heightmaps,
                legs_order,
                simulation_dt,
                ref_base_lin_vel,
                ref_base_ang_vel,
                env.step_num,
                qpos,
                qvel,
                feet_jac,
                feet_jac_dot,
                feet_vel,
                legs_qfrc_passive,
                legs_qfrc_bias,
                legs_mass_matrix,
                legs_qpos_idx,
                legs_qvel_idx,
                tau,
                inertia,
                env.mjData.contact,
            )
            # Limit tau between tau_limits
            for leg in ["FL", "FR", "RL", "RR"]:
                tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
                tau[leg] = np.clip(tau[leg], tau_min, tau_max)

            # Set control and mujoco step -------------------------------------------------------------------------
            action = np.zeros(env.mjModel.nu)
            action[env.legs_tau_idx.FL] = tau.FL
            action[env.legs_tau_idx.FR] = tau.FR
            action[env.legs_tau_idx.RL] = tau.RL
            action[env.legs_tau_idx.RR] = tau.RR


            # Apply the action to the environment and evolve sim --------------------------------------------------
            state, reward, is_terminated, is_truncated, info = env.step(action=action)

            # Get Controller state observables
            ctrl_state = quadrupedpympc_wrapper.get_obs()

            # Store the history of observations and control -------------------------------------------------------
            base_poz_z_err = ctrl_state["ref_base_height"] - base_pos[2]
            ctrl_state["base_poz_z_err"] = base_poz_z_err

            ep_state_history.append(state)
            ep_time.append(env.simulation_time)
            ep_ctrl_state_history.append(ctrl_state)
            
            # Record to MATLAB logger if enabled
            if mat_logger is not None:
                mat_logger.record_step(env.simulation_time, state, ctrl_state)

            # Render only at a certain frequency -----------------------------------------------------------------
            if render and (time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1):
                _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)

                # Plot the swing trajectory
                feet_traj_geom_ids = plot_swing_mujoco(
                    viewer=env.viewer,
                    swing_traj_controller=quadrupedpympc_wrapper.wb_interface.stc,
                    swing_period=quadrupedpympc_wrapper.wb_interface.stc.swing_period,
                    swing_time=LegsAttr(
                        FL=ctrl_state["swing_time"][0],
                        FR=ctrl_state["swing_time"][1],
                        RL=ctrl_state["swing_time"][2],
                        RR=ctrl_state["swing_time"][3],
                    ),
                    lift_off_positions=ctrl_state["lift_off_positions"],
                    nmpc_footholds=ctrl_state["nmpc_footholds"],
                    ref_feet_pos=ctrl_state["ref_feet_pos"],
                    early_stance_detector=quadrupedpympc_wrapper.wb_interface.esd,
                    geom_ids=feet_traj_geom_ids,
                    nominal_feet_pos=quadrupedpympc_wrapper.wb_interface.nominal_feet_pos,
                    adapted_feet_pos=quadrupedpympc_wrapper.wb_interface.adapted_feet_pos,
                )

                # Update and Plot the heightmap
                if qpympc_cfg.simulation_params["visual_foothold_adaptation"] != "blind":
                    # if(stc.check_apex_condition(current_contact, interval=0.01)):
                    for leg_id, leg_name in enumerate(legs_order):
                        data = heightmaps[
                            leg_name
                        ].data  # .update_height_map(ref_feet_pos[leg_name], yaw=env.base_ori_euler_xyz[2])
                        if data is not None:
                            for i in range(data.shape[0]):
                                for j in range(data.shape[1]):
                                    heightmaps[leg_name].geom_ids[i, j] = render_sphere(
                                        viewer=env.viewer,
                                        position=([data[i][j][0][0], data[i][j][0][1], data[i][j][0][2]]),
                                        diameter=0.01,
                                        color=[0, 1, 0, 0.5],
                                        geom_id=heightmaps[leg_name].geom_ids[i, j],
                                    )

                # Plot the GRF
                for leg_id, leg_name in enumerate(legs_order):
                    feet_GRF_geom_ids[leg_name] = render_vector(
                        env.viewer,
                        vector=feet_GRF[leg_name],
                        pos=feet_pos[leg_name],
                        scale=np.linalg.norm(feet_GRF[leg_name]) * 0.005,
                        color=np.array([0, 1, 0, 0.5]),
                        geom_id=feet_GRF_geom_ids[leg_name],
                    )
                # Render frame
                env.render()
                # Capture frame for video recording
                if video_recorder is not None:
                    video_recorder.capture_frame()
                last_render_time = time.time()

            # Reset the environment if the episode is terminated ------------------------------------------------
            if env.step_num >= N_STEPS_PER_EPISODE or is_terminated or is_truncated:
                if is_terminated:
                    print("Environment terminated")
                else:
                    state_obs_history.append(ep_state_history)
                    ctrl_state_history.append(ep_ctrl_state_history)     

                env.reset(random=True)
                quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))

        if h5py_writer is not None:  # Save episode trajectory data to disk.
            ep_obs_history = collate_obs(ep_state_history)  # | collate_obs(ep_ctrl_state_history)
            ep_traj_time = np.asarray(ep_time)[:, np.newaxis]
            h5py_writer.append_trajectory(state_obs_traj=ep_obs_history, time=ep_traj_time)

    # Cleanup video recorder and keyboard listener before closing environment
    if keyboard_stop_event is not None:
        keyboard_stop_event.set()
    if keyboard_thread is not None and keyboard_thread.is_alive():
        keyboard_thread.join(timeout=1.0)
    if video_recorder is not None:
        video_recorder.cleanup()
    
    # Flush MATLAB .mat file if logging is enabled
    if mat_logger is not None:
        mat_logger.flush()

    env.close()
    if h5py_writer is not None:
        return h5py_writer.file_path


def collate_obs(list_of_dicts) -> dict[str, np.ndarray]:
    """Collates a list of dictionaries containing observation names and numpy arrays
    into a single dictionary of stacked numpy arrays.
    """
    if not list_of_dicts:
        raise ValueError("Input list is empty.")

    # Get all keys (assumes all dicts have the same keys)
    keys = list_of_dicts[0].keys()

    # Stack the values per key
    collated = {key: np.stack([d[key] for d in list_of_dicts], axis=0) for key in keys}
    collated = {key: v[:, None] if v.ndim == 1 else v for key, v in collated.items()}
    return collated


if __name__ == "__main__":
    from quadruped_pympc import config as cfg

    qpympc_cfg = cfg
    # Custom changes to the config here:
    pass

    # Run the simulation with the desired configuration.....
    run_simulation(qpympc_cfg=qpympc_cfg)

    # run_simulation(num_episodes=1, render=False)
