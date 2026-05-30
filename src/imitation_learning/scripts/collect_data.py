#!/usr/bin/python3
"""
Imitation learning data collection for the omni-wheel AMR.
Drive the robot with teleop keyboard while this script records
(scan, cmd_vel) pairs to an HDF5 file.

Usage:
  Terminal 1: ros2 launch drl_navigation launch_sim.launch.py
  Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard \
                --ros-args --remap cmd_vel:=cmd_vel
  Terminal 3: python3 collect_data.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

import numpy as np
import h5py
import os
import copy
import math
import time
import threading

# ── shared state ──────────────────────────────────────────────────────────────
N_BINS     = 20       # LiDAR bins (matches imitation_learning model input)
MAX_RANGE  = 12.0
MIN_RANGE  = 0.3

# Stage layout: start = green square, goal = red/purple square
GOAL_X     =  2.5
GOAL_Y     = -2.5
GOAL_DELTA =  0.5   # distance threshold to consider goal reached

lidar_bins  = np.full(N_BINS, MAX_RANGE, dtype=np.float32)
cmd_vel     = np.zeros(2, dtype=np.float32)   # [vx, wz]  (diff drive)
robot_pos   = np.zeros(2, dtype=np.float32)   # [x, y]
_lock       = threading.Lock()


def bin_scan(ranges):
    """Bin 360-ray scan into N_BINS equal sectors, return min range per bin."""
    arr = np.array(ranges, dtype=np.float32)
    arr = np.where(np.isfinite(arr), arr, MAX_RANGE)
    arr = np.clip(arr, MIN_RANGE, MAX_RANGE)
    n   = len(arr)
    bins = np.array([arr[int(i*n/N_BINS):int((i+1)*n/N_BINS)].min()
                     for i in range(N_BINS)], dtype=np.float32)
    return bins


# ── ROS nodes ─────────────────────────────────────────────────────────────────
class ScanNode(Node):
    def __init__(self):
        super().__init__('il_scan_node')
        self.create_subscription(LaserScan, '/scan', self._cb, 1)

    def _cb(self, msg):
        global lidar_bins
        with _lock:
            lidar_bins = bin_scan(msg.ranges)


class OdomNode(Node):
    def __init__(self):
        super().__init__('il_odom_node')
        self.create_subscription(Odometry, '/odom', self._cb, 1)

    def _cb(self, msg):
        global robot_pos
        with _lock:
            robot_pos[0] = msg.pose.pose.position.x
            robot_pos[1] = msg.pose.pose.position.y


class CmdVelNode(Node):
    """Listens to what teleop is publishing and forwards it to the robot."""
    def __init__(self):
        super().__init__('il_cmdvel_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 1)

    def _cb(self, msg):
        global cmd_vel
        with _lock:
            cmd_vel[0] = msg.linear.x
            cmd_vel[1] = msg.angular.z


class GazeboServices(Node):
    def __init__(self):
        super().__init__('il_gazebo_node')
        self.unpause = self.create_client(Empty, '/unpause_physics')
        self.pause   = self.create_client(Empty, '/pause_physics')
        self.reset   = self.create_client(Empty, '/reset_world')
        for cli in (self.unpause, self.pause, self.reset):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {cli.srv_name}...')

    def call(self, client):
        fut = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)


# ── data collection loop ──────────────────────────────────────────────────────
def collect(total_episodes=100, max_steps=150, save_path=None):
    if save_path is None:
        data_dir  = os.path.join(os.path.dirname(__file__), '..', 'data')
        os.makedirs(data_dir, exist_ok=True)
        save_path = os.path.join(data_dir, 'training_data.hdf5')

    rclpy.init()
    scan_node  = ScanNode()
    odom_node  = OdomNode()
    cmd_node   = CmdVelNode()
    gz         = GazeboServices()

    executor = rclpy.executors.MultiThreadedExecutor()
    for n in (scan_node, odom_node, cmd_node, gz):
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    obs_list, next_obs_list, action_list = [], [], []
    reward_list, done_list, timeout_list = [], [], []

    print(f"\n=== Imitation Learning Data Collection ===")
    print(f"Episodes: {total_episodes}  |  Max steps: {max_steps}")
    print(f"Drive with teleop keyboard. Ctrl+C to stop early.\n")

    try:
        for ep in range(total_episodes):
            gz.call(gz.reset)
            gz.call(gz.unpause)
            time.sleep(0.5)

            with _lock:
                obs = np.concatenate([lidar_bins.copy(), robot_pos.copy()])
            step = 0
            ep_reward = 0.0

            while step < max_steps:
                # record state before action
                with _lock:
                    current_obs = np.concatenate([lidar_bins.copy(), robot_pos.copy()])
                    action = cmd_vel.copy()

                gz.call(gz.unpause)
                time.sleep(0.1)

                with _lock:
                    next_obs_arr = np.concatenate([lidar_bins.copy(), robot_pos.copy()])
                    min_scan = lidar_bins.min()

                # reward / done logic
                collision = min_scan < 0.35
                dist_to_goal = np.hypot(
                    next_obs_arr[N_BINS]     - GOAL_X,
                    next_obs_arr[N_BINS + 1] - GOAL_Y)
                goal_reached = dist_to_goal < GOAL_DELTA

                if collision:
                    reward = -100.0
                elif goal_reached:
                    reward = 100.0
                else:
                    reward = -0.01 - dist_to_goal * 0.01   # small shaping
                done    = collision or goal_reached
                timeout = (step == max_steps - 1)

                obs_list.append(current_obs)
                next_obs_list.append(next_obs_arr)
                action_list.append(action)
                reward_list.append(reward)
                done_list.append(done)
                timeout_list.append(timeout)
                ep_reward += reward

                if done or timeout:
                    reason = "COLLISION" if collision else ("GOAL" if goal_reached else "TIMEOUT")
                    print(f"Episode {ep:3d} | steps={step:4d} | "
                          f"reward={ep_reward:8.2f} | {reason}")
                    break
                step += 1

            gz.call(gz.pause)

    except KeyboardInterrupt:
        print("\nStopping early — saving collected data...")

    # save
    n_samples = len(obs_list)
    print(f"\nSaving {n_samples} transitions to {save_path}")
    with h5py.File(save_path, 'w') as hf:
        hf.create_dataset('observations',      data=np.array(obs_list,      dtype=np.float32))
        hf.create_dataset('next_observations', data=np.array(next_obs_list, dtype=np.float32))
        hf.create_dataset('actions',           data=np.array(action_list,   dtype=np.float32))
        hf.create_dataset('rewards',           data=np.array(reward_list,   dtype=np.float32))
        hf.create_dataset('terminals',         data=np.array(done_list,     dtype=bool))
        hf.create_dataset('timeouts',          data=np.array(timeout_list,  dtype=bool))
    print("Done.")

    rclpy.shutdown()


if __name__ == '__main__':
    collect(total_episodes=100, max_steps=150)
