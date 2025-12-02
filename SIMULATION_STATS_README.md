# Simulation Rollout Statistics Collection

This enhanced simulation script now collects detailed performance statistics for rollout analysis.

## New Features

### Data Collection
- **Iterations per second**: Real-time simulation performance tracking
- **Episode timing**: Complete episode duration measurement
- **Base velocity statistics**: Average and standard deviation of robot velocity during episodes
- **Success rate**: Percentage of episodes completed without early termination

### Statistics Output
The simulation now provides comprehensive rollout statistics including:
- Per-episode metrics (timing, velocity, success)
- Aggregated statistics with mean and standard deviation
- Success rate analysis
- Automatic saving to JSON format

### Usage Example

```python
from quadruped_pympc import config as cfg
from simulation.simulation import run_simulation

# Run simulation with statistics collection
result = run_simulation(
    qpympc_cfg=cfg,
    num_episodes=10,
    num_seconds_per_episode=30,
    render=False,
    recording_path="./rollout_data"
)

# Access results
if isinstance(result, tuple):
    data_path, stats = result
    print(f"Success rate: {stats.success_rate:.1%}")
    print(f"Avg velocity: {stats.avg_base_velocity:.3f} ± {stats.std_base_velocity:.3f} m/s")
    print(f"Avg iterations/sec: {stats.avg_iterations_per_second:.1f}")
```

### Output Format

#### Console Output
```
ROLLOUT STATISTICS SUMMARY
============================================================
Number of episodes: 10
Success rate: 90.0%
Average episode time: 12.34 ± 1.23 s
Average iterations/sec: 95.2 ± 8.1
Average base velocity: 1.234 ± 0.156 m/s
============================================================
```

#### JSON Output (saved to `rollout_statistics.json`)
```json
{
  "num_episodes": 10,
  "avg_episode_time": 12.34,
  "std_episode_time": 1.23,
  "avg_iterations_per_second": 95.2,
  "std_iterations_per_second": 8.1,
  "avg_base_velocity": 1.234,
  "std_base_velocity": 0.156,
  "success_rate": 0.9,
  "episode_stats": [...]
}
```

### Data Structure

#### EpisodeStats
- `episode_num`: Episode identifier
- `episode_time`: Total episode duration (seconds)
- `iterations_per_second`: Simulation steps per wall-clock second
- `avg_base_velocity`: Mean horizontal velocity during episode (m/s)
- `base_velocity_std`: Standard deviation of velocity during episode
- `total_steps`: Number of simulation steps completed
- `successful`: Whether episode completed without early termination

#### RolloutStats
- Aggregated statistics across all episodes
- Includes mean and standard deviation for key metrics
- Contains list of individual episode statistics

## Benefits for Analysis

1. **Performance Benchmarking**: Track simulation performance across different configurations
2. **Behavior Analysis**: Analyze velocity patterns and consistency
3. **Robustness Assessment**: Monitor success rates under various conditions
4. **Comparative Studies**: Compare different controller parameters or robot configurations