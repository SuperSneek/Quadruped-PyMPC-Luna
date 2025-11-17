import rclpy 
from rclpy.node import Node 
from dls2_msgs.msg import BaseStateMsg, BlindStateMsg, ControlSignalMsg, TrajectoryGeneratorMsg
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import threading
import multiprocessing


import copy

# Gym and Simulation related imports
import mujoco
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.quadruped_utils import LegsAttr

# Helper functions for plotting
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from gym_quadruped.utils.mujoco.visual import render_vector
from gym_quadruped.utils.mujoco.visual import render_sphere

# Config imports
from quadruped_pympc import config as cfg

import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

USE_SCHEDULER = True # Use the scheduler to compute the control signal
SCHEDULER_FREQ = 500 # Frequency of the scheduler
RENDER_FREQ = 30

# Shell for the controllers ----------------------------------------------
class Simulator_Node(Node):
    def __init__(self):
        super().__init__('Simulator_Node')

        # Subscribers and Publishers
        self.publisher_base_state = self.create_publisher(BaseStateMsg,"/dls2/base_state", 1)
        self.publisher_blind_state = self.create_publisher(BlindStateMsg,"/dls2/blind_state", 1)
        self.publisher_imu = self.create_publisher(Imu, "/imu", 1)
        self.publisher_odom = self.create_publisher(Odometry, "/odom", 1)
        self.publisher_joint_states = self.create_publisher(JointState, "/joint_states", 1)
        
        # Contact publishers for each foot
        self.publisher_contact_FL = self.create_publisher(Bool, "/contact/FL", 1)
        self.publisher_contact_FR = self.create_publisher(Bool, "/contact/FR", 1)
        self.publisher_contact_RL = self.create_publisher(Bool, "/contact/RL", 1)
        self.publisher_contact_RR = self.create_publisher(Bool, "/contact/RR", 1)
        
        self.subscriber_control_signal = self.create_subscription(ControlSignalMsg,"dls2/quadruped_pympc_torques", self.get_torques_callback, 1)
        self.subscriber_trajectory_generator = self.create_subscription(TrajectoryGeneratorMsg,"dls2/trajectory_generator", self.get_trajectory_generator_callback, 1)

        self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_simulator_step_callback)

        # Timing stuff
        self.loop_time = 0.002
        self.last_start_time = None
        self.last_mpc_loop_time = 0.0


        # Mujoco env
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=1.0/SCHEDULER_FREQ,
            base_vel_command_type="human"
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
        

        self.last_render_time = time.time()
        self.env.render()  

        # Extract joint names in order [FL, FR, RL, RR]
        self.joint_names = []
        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            self.joint_names.extend(self.env.robot_cfg.leg_joints[leg_name])

        # Torque vector
        self.desired_tau = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])

        # Desired PD 
        self.desired_joints_position = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
        self.desired_joints_velocity = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])


    def get_torques_callback(self, msg):
        
        torques = np.array(msg.torques)

        self.desired_tau.FL = torques[0:3]
        self.desired_tau.FR = torques[3:6]
        self.desired_tau.RL = torques[6:9]
        self.desired_tau.RR = torques[9:12]




    def get_trajectory_generator_callback(self, msg):

        joints_position = np.array(msg.joints_position)

        self.desired_joints_position.FL = joints_position[0:3]
        self.desired_joints_position.FR = joints_position[3:6]
        self.desired_joints_position.RL = joints_position[6:9]
        self.desired_joints_position.RR = joints_position[9:12]
        


    def compute_simulator_step_callback(self):

        action = np.zeros(self.env.mjModel.nu)
        action[self.env.legs_tau_idx.FL] = self.desired_tau.FL.reshape(-1)
        action[self.env.legs_tau_idx.FR] = self.desired_tau.FR.reshape(-1)
        action[self.env.legs_tau_idx.RL] = self.desired_tau.RL.reshape(-1)
        action[self.env.legs_tau_idx.RR] = self.desired_tau.RR.reshape(-1)
        self.env.step(action=action)

        base_lin_vel = self.env.base_lin_vel(frame='world')
        base_ang_vel = self.env.base_ang_vel(frame='base')
        base_pos = self.env.base_pos

        base_state_msg = BaseStateMsg()
        base_state_msg.position = base_pos
        base_state_msg.orientation = np.roll(self.env.mjData.qpos[3:7],-1)
        base_state_msg.linear_velocity = base_lin_vel
        base_state_msg.angular_velocity = base_ang_vel
        self.publisher_base_state.publish(base_state_msg)

        blind_state_msg = BlindStateMsg()
        blind_state_msg.joints_position = self.env.mjData.qpos[7:]
        blind_state_msg.joints_velocity = self.env.mjData.qvel[6:]
        self.publisher_blind_state.publish(blind_state_msg)

        # Publish Joint States
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.env.mjData.qpos[7:].tolist()
        joint_state_msg.velocity = self.env.mjData.qvel[6:].tolist()
        joint_state_msg.effort = action.tolist()
        self.publisher_joint_states.publish(joint_state_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        
        # Orientation (quaternion)
        imu_msg.orientation.x = self.env.mjData.qpos[3]
        imu_msg.orientation.y = self.env.mjData.qpos[4]
        imu_msg.orientation.z = self.env.mjData.qpos[5]
        imu_msg.orientation.w = self.env.mjData.qpos[6]
        
        # Angular velocity (in body frame)
        imu_msg.angular_velocity.x = base_ang_vel[0]
        imu_msg.angular_velocity.y = base_ang_vel[1]
        imu_msg.angular_velocity.z = base_ang_vel[2]
        
        # Linear acceleration (get from sensor data)
        # MuJoCo provides sensor accelerometer data if available
        imu_msg.linear_acceleration.x = self.env.mjData.qacc[0]
        imu_msg.linear_acceleration.y = self.env.mjData.qacc[1]
        imu_msg.linear_acceleration.z = self.env.mjData.qacc[2] + cfg.gravity_constant
        
        self.publisher_imu.publish(imu_msg)

        # Publish Odometry data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Position
        odom_msg.pose.pose.position.x = base_pos[0]
        odom_msg.pose.pose.position.y = base_pos[1]
        odom_msg.pose.pose.position.z = base_pos[2]
        
        # Orientation
        odom_msg.pose.pose.orientation.x = self.env.mjData.qpos[3]
        odom_msg.pose.pose.orientation.y = self.env.mjData.qpos[4]
        odom_msg.pose.pose.orientation.z = self.env.mjData.qpos[5]
        odom_msg.pose.pose.orientation.w = self.env.mjData.qpos[6]
        
        # Linear velocity (in world frame)
        odom_msg.twist.twist.linear.x = base_lin_vel[0]
        odom_msg.twist.twist.linear.y = base_lin_vel[1]
        odom_msg.twist.twist.linear.z = base_lin_vel[2]
        
        # Angular velocity (in body frame)
        odom_msg.twist.twist.angular.x = base_ang_vel[0]
        odom_msg.twist.twist.angular.y = base_ang_vel[1]
        odom_msg.twist.twist.angular.z = base_ang_vel[2]
        
        self.publisher_odom.publish(odom_msg)

        # Publish foot contact states
        contact_state, _ = self.env.feet_contact_state()
        
        contact_fl_msg = Bool()
        contact_fl_msg.data = bool(contact_state.FL)
        self.publisher_contact_FL.publish(contact_fl_msg)
        
        contact_fr_msg = Bool()
        contact_fr_msg.data = bool(contact_state.FR)
        self.publisher_contact_FR.publish(contact_fr_msg)
        
        contact_rl_msg = Bool()
        contact_rl_msg.data = bool(contact_state.RL)
        self.publisher_contact_RL.publish(contact_rl_msg)
        
        contact_rr_msg = Bool()
        contact_rr_msg.data = bool(contact_state.RR)
        self.publisher_contact_RR.publish(contact_rr_msg)


        # Render only at a certain frequency -----------------------------------------------------------------
        if time.time() - self.last_render_time > 1.0 / RENDER_FREQ:
            self.env.render()
            self.last_render_time = time.time()


def main():
    print('Hello from the gym_quadruped simulator.')
    rclpy.init()

    simulator_node = Simulator_Node()

    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
