import rclpy 
from rclpy.node import Node 
from dls2_interface.msg import BaseState, BlindState
from sensor_msgs.msg import JointState

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import mujoco

# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv

# Config imports
from quadruped_pympc import config as cfg

import os 
import argparse

dir_path = os.path.dirname(os.path.realpath(__file__))

RENDER_FREQ = 30  # Hz

class StateVisualizer(Node):
    def __init__(self, joint_topic='/blind_state', base_topic='/base_state', use_joint_state_msg=False):
        super().__init__('StateVisualizer')
        
        # Parameters
        self.use_joint_state_msg = use_joint_state_msg
        
        self.get_logger().info(f'Visualizing robot state from topics:')
        self.get_logger().info(f'  Joint states: {joint_topic}')
        self.get_logger().info(f'  Base state: {base_topic}')
        self.get_logger().info(f'  Message type: {"sensor_msgs/JointState" if use_joint_state_msg else "BlindState"}')

        # Subscribers
        if use_joint_state_msg:
            self.subscriber_joint_state = self.create_subscription(
                JointState, joint_topic, self.joint_state_callback, 1)
        else:
            self.subscriber_joint_state = self.create_subscription(
                BlindState, joint_topic, self.blind_state_callback, 1)
        
        self.subscriber_base_state = self.create_subscription(
            BaseState, base_topic, self.base_state_callback, 1)

        # Timer for rendering
        self.timer = self.create_timer(1.0/RENDER_FREQ, self.render_callback)

        # State storage
        self.joint_positions = None
        self.joint_velocities = None
        self.base_position = np.array([0.0, 0.0, 0.3])
        self.base_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        
        self.received_joint_data = False
        self.received_base_data = False

        # Mujoco env for visualization
        self.get_logger().info('Initializing MuJoCo environment...')
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=1.0/RENDER_FREQ,  # Doesn't matter much since we're not simulating
            base_vel_command_type="human"
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
        
        # Initialize viewer
        self.env.render()  
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        
        self.get_logger().info('State visualizer ready!')


    def blind_state_callback(self, msg):
        """Callback for BlindState message"""
        self.joint_positions = np.array(msg.joints_position)
        self.joint_velocities = np.array(msg.joints_velocity)
        self.received_joint_data = True


    def joint_state_callback(self, msg):
        """Callback for sensor_msgs/JointState message"""
        self.joint_positions = np.array(msg.position)
        self.joint_velocities = np.array(msg.velocity) if len(msg.velocity) > 0 else np.zeros_like(msg.position)
        self.received_joint_data = True


    def base_state_callback(self, msg):
        """Callback for BaseState message"""
        self.base_position = np.array(msg.pose.position)
        # Orientation in msg is [x, y, z, w], need to convert to [w, x, y, z] for mujoco
        quat_xyzw = np.array(msg.pose.orientation)
        self.base_orientation = np.roll(quat_xyzw, 1)  # [w, x, y, z]
        self.base_lin_vel = np.array(msg.velocity.linear)
        self.base_ang_vel = np.array(msg.velocity.angular)
        self.received_base_data = True


    def render_callback(self):
        """Update the visualization with the latest state"""
        
        if not self.received_joint_data:
            if not hasattr(self, '_warned_no_joint_data'):
                self.get_logger().warn('No joint state data received yet...', throttle_duration_sec=2.0)
                self._warned_no_joint_data = True
            return
        
        # Update the mujoco model state
        # qpos structure: [base_pos(3), base_quat(4), joint_pos(12)]
        # qvel structure: [base_lin_vel(3), base_ang_vel(3), joint_vel(12)]
        
        # Update base position and orientation
        self.env.mjData.qpos[0:3] = self.base_position
        self.env.mjData.qpos[3:7] = self.base_orientation
        
        # Update joint positions
        if self.joint_positions is not None:
            n_joints = min(len(self.joint_positions), self.env.mjModel.nq - 7)
            self.env.mjData.qpos[7:7+n_joints] = self.joint_positions[:n_joints]
        
        # Update base velocities
        self.env.mjData.qvel[0:3] = self.base_lin_vel
        self.env.mjData.qvel[3:6] = self.base_ang_vel
        
        # Update joint velocities
        if self.joint_velocities is not None:
            n_joints = min(len(self.joint_velocities), self.env.mjModel.nv - 6)
            self.env.mjData.qvel[6:6+n_joints] = self.joint_velocities[:n_joints]
        
        # Forward kinematics to update dependent quantities
        mujoco.mj_forward(self.env.mjModel, self.env.mjData)
        
        # Render
        try:
            self.env.render()
        except Exception as e:
            self.get_logger().error(f'Render error: {e}')


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Visualize quadruped state from ROS2 topics')
    parser.add_argument('--joint-topic', type=str, default='/blind_state',
                        help='Topic name for joint states (default: /blind_state)')
    parser.add_argument('--base-topic', type=str, default='/base_state',
                        help='Topic name for base state (default: /base_state)')
    parser.add_argument('--use-joint-state-msg', action='store_true',
                        help='Use sensor_msgs/JointState instead of BlindState')
    
    parsed_args, remaining = parser.parse_known_args()
    
    print('State Visualizer - Visualizing quadruped state from ROS2 topics')
    print(f'Joint topic: {parsed_args.joint_topic}')
    print(f'Base topic: {parsed_args.base_topic}')
    
    rclpy.init(args=remaining)

    visualizer_node = StateVisualizer(
        joint_topic=parsed_args.joint_topic,
        base_topic=parsed_args.base_topic,
        use_joint_state_msg=parsed_args.use_joint_state_msg
    )

    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        print('\nShutting down visualizer...')
    finally:
        visualizer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
