#!/usr/bin/python3
"""
Imitation learning data collection.
Drive from green square to red square. Episode ends only on
collision or goal reached. Obstacles randomise each episode.

Terminal 1: ros2 launch imitation_learning imitation_learning.launch.py
Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard
Terminal 3: python3 collect_data.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState

import numpy as np
import h5py, os, time, threading

# ── constants ──────────────────────────────────────────────────────────────────
N_BINS     = 20
MAX_RANGE  = 12.0
MIN_RANGE  = 0.3

START_X, START_Y = -1.8,  1.8   # green square
GOAL_X,  GOAL_Y  =  2.0, -2.0   # red square
GOAL_DELTA       =  0.5

# 14 obstacle names from the world file
BOX_NAMES = [f'box{i}' for i in range(1, 15)]

# Stage area where blocks can be placed (inside stage, away from edges)
STAGE_X = (-2.0, 2.0)
STAGE_Y = (-2.0, 2.0)
BLOCK_Z  = 0.1   # height from world file

# ── shared state ──────────────────────────────────────────────────────────────
lidar_bins = np.full(N_BINS, MAX_RANGE, dtype=np.float32)
cmd_vel    = np.zeros(2, dtype=np.float32)
robot_pos  = np.zeros(2, dtype=np.float32)
_lock      = threading.Lock()


def bin_scan(ranges):
    arr = np.array(ranges, dtype=np.float32)
    arr = np.where(np.isfinite(arr) & (arr > MIN_RANGE), arr, MAX_RANGE)
    arr = np.clip(arr, MIN_RANGE, MAX_RANGE)
    n   = len(arr)
    return np.array([arr[int(i*n/N_BINS):int((i+1)*n/N_BINS)].min()
                     for i in range(N_BINS)], dtype=np.float32)


# ── ROS nodes ─────────────────────────────────────────────────────────────────
class ScanNode(Node):
    def __init__(self):
        super().__init__('il_scan_node')
        self.create_subscription(LaserScan, '/scan', self._cb, 1)
    def _cb(self, msg):
        global lidar_bins
        with _lock: lidar_bins = bin_scan(msg.ranges)

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
    """Records teleop commands AND publishes zero if key released for > 0.15s."""
    def __init__(self):
        super().__init__('il_cmdvel_node')
        self._pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._last = time.time()
        self.create_subscription(Twist, '/cmd_vel', self._cb, 1)
        self.create_timer(0.05, self._watchdog)

    def _cb(self, msg):
        global cmd_vel
        self._last = time.time()
        with _lock:
            cmd_vel[0] = msg.linear.x
            cmd_vel[1] = msg.angular.z

    def _watchdog(self):
        if time.time() - self._last > 0.15:
            self._pub.publish(Twist())   # explicit zero stops the robot


class GazeboServices(Node):
    def __init__(self):
        super().__init__('il_gazebo_node')
        self.unpause   = self.create_client(Empty, '/unpause_physics')
        self.pause     = self.create_client(Empty, '/pause_physics')
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        for cli in (self.unpause, self.pause, self.set_state):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {cli.srv_name}...')

    def unpause_physics(self):
        fut = self.unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def pause_physics(self):
        fut = self.pause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def move_entity(self, name, x, y, z=BLOCK_Z):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0  # perfectly upright, no rotation
        fut = self.set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def teleport_robot(self):
        self.move_entity('my_bot', START_X, START_Y, z=0.15)

    def randomise_blocks(self):
        """Move all 14 blocks to random non-overlapping positions."""
        rng      = np.random.default_rng()
        placed   = [(START_X, START_Y), (GOAL_X, GOAL_Y)]
        min_sep  = 0.85

        for name in BOX_NAMES:
            for _ in range(200):
                x = rng.uniform(*STAGE_X)
                y = rng.uniform(*STAGE_Y)
                if all(np.hypot(x-px, y-py) >= min_sep for px, py in placed):
                    placed.append((x, y))
                    self.move_entity(name, x, y)  # no rotation, upright
                    break


# ── data collection loop ──────────────────────────────────────────────────────
def collect(total_episodes=200, save_path=None):
    if save_path is None:
        data_dir  = os.path.join(os.path.dirname(__file__), '..', 'data')
        os.makedirs(data_dir, exist_ok=True)
        save_path = os.path.join(data_dir, 'training_data.hdf5')

    rclpy.init()
    nodes = [ScanNode(), OdomNode(), CmdVelNode(), GazeboServices()]
    gz    = nodes[-1]

    executor   = rclpy.executors.MultiThreadedExecutor()
    for n in nodes: executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()

    obs_list, next_obs_list, action_list = [], [], []
    reward_list, done_list = [], []

    print("\n=== Imitation Learning Data Collection ===")
    print(f"Green ({START_X},{START_Y}) -> Red ({GOAL_X},{GOAL_Y})")
    print("Drive to the red square. Blocks randomise each episode.")
    print("Ctrl+C to stop and save.\n")

    try:
        for ep in range(total_episodes):
            # Pause physics → move blocks + robot → unpause
            # (pausing prevents blocks from falling during repositioning)
            gz.pause_physics()
            gz.randomise_blocks()
            gz.teleport_robot()
            time.sleep(0.2)
            gz.unpause_physics()
            time.sleep(0.3)

            step      = 0
            ep_reward = 0.0

            while True:   # only collision or goal ends the episode
                with _lock:
                    current_obs = np.concatenate([lidar_bins.copy(), robot_pos.copy()])
                    action      = cmd_vel.copy()

                time.sleep(0.1)

                with _lock:
                    next_obs    = np.concatenate([lidar_bins.copy(), robot_pos.copy()])
                    min_scan    = lidar_bins.min()

                dist_to_goal = np.hypot(next_obs[N_BINS]   - GOAL_X,
                                        next_obs[N_BINS+1] - GOAL_Y)
                collision    = min_scan < 0.25
                goal_reached = dist_to_goal < GOAL_DELTA

                reward = (-100.0 if collision else
                          100.0  if goal_reached else
                          -0.01 - dist_to_goal * 0.01)

                obs_list.append(current_obs)
                next_obs_list.append(next_obs)
                action_list.append(action)
                reward_list.append(reward)
                done_list.append(collision or goal_reached)
                ep_reward += reward
                step      += 1

                if collision or goal_reached:
                    reason = "COLLISION" if collision else "GOAL"
                    print(f"Ep {ep:3d} | steps={step:4d} | "
                          f"reward={ep_reward:8.2f} | {reason}")
                    break

    except KeyboardInterrupt:
        print("\nStopping — saving data...")

    n = len(obs_list)
    print(f"Saving {n} transitions to {save_path}")
    with h5py.File(save_path, 'w') as hf:
        hf.create_dataset('observations',      data=np.array(obs_list,      dtype=np.float32))
        hf.create_dataset('next_observations', data=np.array(next_obs_list, dtype=np.float32))
        hf.create_dataset('actions',           data=np.array(action_list,   dtype=np.float32))
        hf.create_dataset('rewards',           data=np.array(reward_list,   dtype=np.float32))
        hf.create_dataset('terminals',         data=np.array(done_list,     dtype=bool))
    print("Done.")
    rclpy.shutdown()


if __name__ == '__main__':
    collect(total_episodes=200)
