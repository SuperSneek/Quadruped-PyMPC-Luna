"""
ROS2 node that subscribes to /joint_states topic and visualizes the robot model
with the received joint states in real-time.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
import time
import mujoco

from gym_quadruped.quadruped_env import QuadrupedEnv
from quadruped_pympc import config as cfg
from dls2_msgs.msg import BlindStateMsg


class RobotVisualizer(Node):
    """
    ROS2 node that visualizes a quadruped robot based on joint states.
    Subscribes to /joint_states topic and updates the visualization in real-time.
    """

    def __init__(self):
        super().__init__("robot_visualizer_node")

        # Create the quadruped environment for visualization
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params["scene"],
            sim_dt=0.001,  # Small timestep for accurate visualization
            base_vel_command_type="human",
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
        self.env.render()

        # Subscribe to joint_states topic
        self.subscription = self.create_subscription(
            BlindStateMsg,
            "/dls2/blind_state",
            self.joint_states_callback,
            qos_profile=1,
        )

        # Timing for rendering
        self.last_render_time = time.time()
        self.render_freq = 30  # Hz
        self.received_joint_states = False

        self.get_logger().info(
            f"Robot Visualizer initialized for robot: {cfg.robot}"
        )

    def joint_states_callback(self, msg: BlindStateMsg):
        """
        Callback function that receives joint states and updates the visualization.

        Args:
            msg (BlindStateMsg): The joint state message containing joint positions and velocities.
        """
        try:
            # Update the base position and orientation from the environment
            # (assuming they remain constant or are updated elsewhere)
            current_qpos = self.env.mjData.qpos.copy()
            current_qvel = self.env.mjData.qvel.copy()

            # Map received joint states to the correct indices
            # Assuming the joint state message contains all joint positions
            if len(msg.joints_position) > 0:
                # Update joint positions
                # Skip the first 7 states (base position + orientation)
                num_joints = min(len(msg.joints_position), len(current_qpos) - 7)
                current_qpos[7 : 7 + num_joints] = msg.joints_position[:num_joints]
                # Update joint velocities if available
                if len(msg.joints_velocity) > 0:
                    num_velocities = min(len(msg.joints_velocity), len(current_qvel) - 6)
                    current_qvel[6 : 6 + num_velocities] = msg.joints_velocity[:num_velocities]

                # Update the mujoco model
                self.env.mjData.qpos[:] = current_qpos
                self.env.mjData.qvel[:] = current_qvel

                # Forward dynamics to update the model state
                mujoco.mj_forward(self.env.mjModel, self.env.mjData)

                self.received_joint_states = True

                # Render at the specified frequency
                current_time = time.time()
                if current_time - self.last_render_time > 1.0 / self.render_freq:
                    self.env.render()
                    self.last_render_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {str(e)}")

    def run(self):
        """Main loop to keep the node alive."""
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down robot visualizer...")
        finally:
            self.env.close()
            self.destroy_node()


def main():
    print("Starting Robot Visualizer Node...")
    rclpy.init()

    robot_visualizer = RobotVisualizer()
    robot_visualizer.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
