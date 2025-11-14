#!/bin/bash
# Tmux session configuration for Quadruped-PyMPC-Luna
# This script creates a tmux session with multiple windows

SESSION_NAME="quadruped_dev"
MPC_HOME_PATH="/home/jonas/Documents/Kamaro/Quadruped-PyMPC-Luna"
LUNA_WS_PATH="/home/jonas/Documents/Kamaro/kamarovision/luna_ws"
STATE_ESTIMATOR_PATH="/home/jonas/Documents/Kamaro/muse_ros2/muse_ws"

# Check if session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    # Create new session (detached)
    tmux new-session -d -s $SESSION_NAME -n "main"

    # Window 1: Main - Luna ws
    tmux send-keys -t $SESSION_NAME:0 "cd $LUNA_WS_PATH" C-m
    tmux send-keys -t $SESSION_NAME:0 "source $LUNA_WS_PATH/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0 "ros2 launch mujoco_sim mujoco_sim.launch.py use_viewer:=true" C-m
    
    # Window 2: Controller - Source workspace 2 and run controller
    tmux new-window -t $SESSION_NAME:1 -n "controller"
    tmux send-keys -t $SESSION_NAME:1 "cd $MPC_HOME_PATH" C-m
    tmux send-keys -t $SESSION_NAME:1 "source $MPC_HOME_PATH/.venv/bin/activate" C-m
    tmux send-keys -t $SESSION_NAME:1 "source $MPC_HOME_PATH/ros2/msgs_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1 "python $MPC_HOME_PATH/ros2/run_controller.py" C-m
    
    # Window 3: Simulator - Source workspace 3 and run simulator
    tmux new-window -t $SESSION_NAME:2 -n "simulator"
    tmux send-keys -t $SESSION_NAME:2 "cd $MPC_HOME_PATH/ros2" C-m
    
    # Split window vertically
    tmux split-window -h -t $SESSION_NAME:2
    
    # Left pane (index 0) - Simulator
    tmux send-keys -t $SESSION_NAME:2.0 "cd $STATE_ESTIMATOR_PATH" C-m
    tmux send-keys -t $SESSION_NAME:2.0 "source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:2.0 "ros2 launch state_estimator state_estimator.launch.py use_contact_data_packaging:=true" C-m
    
    # Right pane (index 1) - Monitoring
    tmux send-keys -t $SESSION_NAME:2.1 "cd $MPC_HOME_PATH/ros2" C-m
    tmux send-keys -t $SESSION_NAME:2.1 "source msgs_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:2.1 "ros2 launch message_converters message_converters" C-m
    
    # Window 4: Console - For monitoring/debugging
    tmux new-window -t $SESSION_NAME:3 -n "console"
    tmux send-keys -t $SESSION_NAME:3 "cd /home/jonas/Documents/Kamaro/Quadruped-PyMPC-Luna/ros2" C-m
    # TODO: Add your workspace source command here
    # tmux send-keys -t $SESSION_NAME:3 "source /path/to/workspace4/setup.bash" C-m
    
    # Window 5: Logs - For viewing rosbags or logs
    tmux new-window -t $SESSION_NAME:4 -n "logs"
    tmux send-keys -t $SESSION_NAME:4 "cd $MPC_HOME_PATH" C-m
    
    # Window 6: Terminal - General purpose terminal
    tmux new-window -t $SESSION_NAME:5 -n "terminal"
    tmux send-keys -t $SESSION_NAME:5 "cd $MPC_HOME_PATH" C-m
    tmux send-keys -t $SESSION_NAME:5 "source $MPC_HOME_PATH/ros2/msgs_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:5 "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
    
    # Select the first window
    tmux select-window -t $SESSION_NAME:0
fi

# Attach to the session
tmux attach-session -t $SESSION_NAME
