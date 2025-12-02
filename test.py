from quadruped_pympc import config as cfg
from simulation.simulation import run_simulation

# Run simulation with statistics collection
result = run_simulation(
    qpympc_cfg=cfg,
    num_episodes=3,
    num_seconds_per_episode=10,
    render=False,
    recording_path="./rollout_data"
)

# Access results
if isinstance(result, tuple):
    data_path, stats = result
    print(f"Success rate: {stats.success_rate:.1%}")
    print(f"Avg velocity: {stats.avg_base_velocity:.3f} ± {stats.std_base_velocity:.3f} m/s")
    print(f"Avg iterations/sec: {stats.avg_iterations_per_second:.1f}")