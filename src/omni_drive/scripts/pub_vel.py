#!/usr/bin/python3

import rclpy
import math
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

axes = np.array([0,0,0], float)

class VelPublisher(Node):

    def __init__(self):
        super().__init__('wheel_vel_publiser')
        
        self.vel = np.array([0,0,0], float)
        self.wheel_vel = np.array([0,0,0,0], float)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.timer_interval = 0.02
        self.L = 0.125 # distance from the robot center to the wheel
        self.Rw = 0.03 # Radius ot the wheel

        self.linear_vel = 1
        self.omega = 3

        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

    def timer_callback(self):
        global axes

        self.vel[0] = axes[0]*self.linear_vel
        self.vel[1] = axes[1]*self.linear_vel
        self.vel[2] = axes[2]*self.omega

        self.wheel_vel[0] = (self.vel[0]*math.sin(math.pi/4            ) + self.vel[1]*math.cos(math.pi/4            ) + self.L*self.vel[2])/self.Rw
        self.wheel_vel[1] = (self.vel[0]*math.sin(math.pi/4 + math.pi/2) + self.vel[1]*math.cos(math.pi/4 + math.pi/2) + self.L*self.vel[2])/self.Rw
        self.wheel_vel[2] = (self.vel[0]*math.sin(math.pi/4 - math.pi)   + self.vel[1]*math.cos(math.pi/4 - math.pi)   + self.L*self.vel[2])/self.Rw
        self.wheel_vel[3] = (self.vel[0]*math.sin(math.pi/4 - math.pi/2) + self.vel[1]*math.cos(math.pi/4 - math.pi/2) + self.L*self.vel[2])/self.Rw

        array_forPublish = Float64MultiArray(data=self.wheel_vel)    
        #rclpy.logging._root_logger.info(f"wheel vel : {self.wheel_vel}")
        self.publisher_.publish(array_forPublish)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('omni_drive_joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global axes

        axes[0] = -data.axes[0]
        axes[1] = -data.axes[1]
        axes[2] = -data.axes[3]

if __name__ == '__main__':
    rclpy.init(args=None)
    
    vel_publisher = VelPublisher()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(vel_publisher)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = vel_publisher.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

