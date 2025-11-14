# Tmux Session Configuration

This folder contains tmux configuration scripts for managing the Quadruped-PyMPC-Luna development environment.

## Files

- `session.sh`: Main tmux session startup script

## Usage

To start the tmux session:

```bash
./.tmux/session.sh
```

Or from anywhere:

```bash
bash /home/jonas/Documents/Kamaro/Quadruped-PyMPC-Luna/.tmux/session.sh
```

## Windows Layout

The script creates a tmux session called `quadruped_dev` with the following windows:

1. **main** (Window 0): Main workspace directory
2. **controller** (Window 1): For running the controller script
3. **simulator** (Window 2): For running the simulator script
4. **console** (Window 3): For monitoring/debugging
5. **logs** (Window 4): For viewing rosbags or logs
6. **terminal** (Window 5): General purpose terminal

## Configuration

Edit `session.sh` to:

1. **Add workspace sourcing**: Uncomment and modify the `source` commands to source your ROS2 workspaces
   ```bash
   tmux send-keys -t $SESSION_NAME:0 "source /path/to/workspace/setup.bash" C-m
   ```

2. **Configure scripts**: Uncomment and modify the script execution commands
   ```bash
   tmux send-keys -t $SESSION_NAME:1 "python run_controller.py" C-m
   ```

3. **Add/Remove windows**: Add new windows or remove existing ones as needed
   ```bash
   tmux new-window -t $SESSION_NAME:6 -n "my_window"
   ```

## Tmux Commands

Useful tmux commands:

- `Ctrl+b d`: Detach from session
- `Ctrl+b n`: Next window
- `Ctrl+b p`: Previous window
- `Ctrl+b 0-9`: Switch to window by number
- `Ctrl+b c`: Create new window
- `Ctrl+b ,`: Rename current window
- `Ctrl+b &`: Kill current window
- `tmux ls`: List all sessions
- `tmux attach -t quadruped_dev`: Attach to session
- `tmux kill-session -t quadruped_dev`: Kill the session

## Example Configuration

Here's an example of how to configure a window to source a workspace and run a script:

```bash
# Window for controller
tmux new-window -t $SESSION_NAME:1 -n "controller"
tmux send-keys -t $SESSION_NAME:1 "cd /home/jonas/Documents/Kamaro/Quadruped-PyMPC-Luna/ros2" C-m
tmux send-keys -t $SESSION_NAME:1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:1 "source ~/ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:1 "python run_controller.py" C-m
```
