# Description: This script is used to simulate the full model of the robot in mujoco
import pathlib

# Authors:
# Giulio Turrisi, Daniel Ordonez
import time
from os import PathLike
from pprint import pprint

import numpy as np
import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any

# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.mujoco.visual import render_sphere, render_vector
from gym_quadruped.utils.quadruped_utils import LegsAttr
from tqdm import tqdm

# Helper functions for plotting
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco

# PyMPC controller imports
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper


@dataclass
class EpisodeStats:
    """Statistics for a single episode"""
    episode_num: int
    episode_time: float
    iterations_per_second: float
    avg_base_velocity: float
    base_velocity_std: float
    total_steps: int
    successful: bool


@dataclass
class RolloutStats:
    """Aggregated statistics across all episodes"""
    num_episodes: int
    avg_episode_time: float
    std_episode_time: float
    avg_iterations_per_second: float
    std_iterations_per_second: float
    avg_base_velocity: float
    std_base_velocity: float
    success_rate: float
    episode_stats: List[EpisodeStats]


def compute_rollout_statistics(episode_stats_list: List[EpisodeStats]) -> RolloutStats:
    """Compute aggregated statistics from episode data"""
    if not episode_stats_list:
        raise ValueError("No episode statistics provided")
    
    episode_times = [ep.episode_time for ep in episode_stats_list]
    iterations_per_sec = [ep.iterations_per_second for ep in episode_stats_list]
    base_velocities = [ep.avg_base_velocity for ep in episode_stats_list]
    successes = [ep.successful for ep in episode_stats_list]
    
    return RolloutStats(
        num_episodes=len(episode_stats_list),
        avg_episode_time=float(np.mean(episode_times)),
        std_episode_time=float(np.std(episode_times)),
        avg_iterations_per_second=float(np.mean(iterations_per_sec)),
        std_iterations_per_second=float(np.std(iterations_per_sec)),
        avg_base_velocity=float(np.mean(base_velocities)),
        std_base_velocity=float(np.std(base_velocities)),
        success_rate=float(np.mean(successes)),
        episode_stats=episode_stats_list
    )


def save_rollout_stats(stats: RolloutStats, filepath: str):
    """Save rollout statistics to JSON file"""
    with open(filepath, 'w') as f:
        json.dump(asdict(stats), f, indent=2)


def run_simulation(
    qpympc_cfg,
    process=0,
    num_episodes=500,
    num_seconds_per_episode=60,
    ref_base_lin_vel=(0.0, 4.0),
    ref_base_ang_vel=(-0.4, 0.4),
    friction_coeff=(0.5, 1.0),
    base_vel_command_type="forward",
    seed=0,
    render=False,
    recording_path: PathLike = None,
):
    np.set_printoptions(precision=3, suppress=True)
    np.random.seed(seed)

    robot_name = qpympc_cfg.robot
    hip_height = qpympc_cfg.hip_height
    robot_leg_joints = qpympc_cfg.robot_leg_joints
    robot_feet_geom_names = qpympc_cfg.robot_feet_geom_names
    scene_name = qpympc_cfg.simulation_params["scene"]
    simulation_dt = qpympc_cfg.simulation_params["dt"]

    # Save all observables available.
    state_obs_names = [] #list(QuadrupedEnv.ALL_OBS)  # + list(IMU.ALL_OBS)

    # Create the quadruped robot environment -----------------------------------------------------------
    env = QuadrupedEnv(
        robot=robot_name,
        scene=scene_name,
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
        num_rows_heightmap = 7
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

    # -----------------------------------------------------------------------------------------------------------
    RENDER_FREQ = 30  # Hz
    N_EPISODES = num_episodes
    N_STEPS_PER_EPISODE = int(num_seconds_per_episode // simulation_dt)
    last_render_time = time.time()

    state_obs_history, ctrl_state_history = [], []
    episode_stats_list: List[EpisodeStats] = []
    for episode_num in range(N_EPISODES):
        ep_state_history, ep_ctrl_state_history, ep_time = [], [], []
        episode_start_time = time.time()
        episode_base_velocities = []
        
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
            
            # Collect base velocity for statistics
            base_speed = np.linalg.norm(base_lin_vel[:2])  # Horizontal speed
            episode_base_velocities.append(base_speed)

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

                env.render()
                last_render_time = time.time()

            # Reset the environment if the episode is terminated ------------------------------------------------
            if env.step_num >= N_STEPS_PER_EPISODE or is_terminated or is_truncated:
                # Calculate episode statistics
                episode_end_time = time.time()
                episode_duration = episode_end_time - episode_start_time
                actual_steps = len(ep_state_history)
                iterations_per_second = actual_steps / episode_duration if episode_duration > 0 else 0
                
                # Calculate velocity statistics
                if episode_base_velocities:
                    avg_base_velocity = float(np.mean(episode_base_velocities))
                    base_velocity_std = float(np.std(episode_base_velocities))
                else:
                    avg_base_velocity = 0.0
                    base_velocity_std = 0.0
                
                # Create episode stats
                ep_stats = EpisodeStats(
                    episode_num=episode_num,
                    episode_time=episode_duration,
                    iterations_per_second=iterations_per_second,
                    avg_base_velocity=avg_base_velocity,
                    base_velocity_std=base_velocity_std,
                    total_steps=actual_steps,
                    successful=not is_terminated
                )
                episode_stats_list.append(ep_stats)
                
                state_obs_history.append(ep_state_history)
                ctrl_state_history.append(ep_ctrl_state_history)
                print(f"Episode {episode_num} completed: {iterations_per_second:.1f} iter/s, "
                      f"avg_vel: {avg_base_velocity:.2f}±{base_velocity_std:.2f} m/s")

                env.reset(random=True)
                quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))

                break

            time.sleep(0.01)  # To avoid overloading the CPU

        if h5py_writer is not None:  # Save episode trajectory data to disk.
            ep_obs_history = collate_obs(ep_state_history)  # | collate_obs(ep_ctrl_state_history)
            ep_traj_time = np.asarray(ep_time)[:, np.newaxis]
            h5py_writer.append_trajectory(state_obs_traj=ep_obs_history, time=ep_traj_time)

    env.close()
    
    # Compute and save rollout statistics
    if episode_stats_list:
        rollout_stats = compute_rollout_statistics(episode_stats_list)
        
        # Print summary statistics
        print("\n" + "="*60)
        print("ROLLOUT STATISTICS SUMMARY")
        print("="*60)
        print(f"Number of episodes: {rollout_stats.num_episodes}")
        print(f"Success rate: {rollout_stats.success_rate:.1%}")
        print(f"Average episode time: {rollout_stats.avg_episode_time:.2f} ± {rollout_stats.std_episode_time:.2f} s")
        print(f"Average iterations/sec: {rollout_stats.avg_iterations_per_second:.1f} ± {rollout_stats.std_iterations_per_second:.1f}")
        print(f"Average base velocity: {rollout_stats.avg_base_velocity:.3f} ± {rollout_stats.std_base_velocity:.3f} m/s")
        print("="*60)
        
        # Save statistics to file if recording
        if recording_path is not None:
            stats_path = pathlib.Path(recording_path) / "rollout_statistics.json"
            save_rollout_stats(rollout_stats, str(stats_path))
            print(f"Statistics saved to: {stats_path.absolute()}")
            return h5py_writer.file_path if h5py_writer is not None else None, rollout_stats
        
        return rollout_stats
    
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

    # Run the simulation with the desired configuration and collect statistics
    result = run_simulation(
        qpympc_cfg=qpympc_cfg,
        num_episodes=5,  # Example: 5 episodes for testing
        num_seconds_per_episode=10,  # Example: 10 seconds per episode
        render=False,
        recording_path="./rollout_data"  # Save statistics and data
    )
    
    # Access rollout statistics
    if isinstance(result, tuple):
        data_path, stats = result
        print(f"Data saved to: {data_path}")
        # You can access individual episode stats: stats.episode_stats
    else:
        stats = result
        print("Rollout completed successfully!")
        # Individual episode data: stats.episode_stats
