#!/usr/bin/env python3
"""Watch /detections_3d and publish a grasp target when a stable detection is found."""

import collections
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray


class GraspFromDetection(Node):

    def __init__(self):
        super().__init__('grasp_from_detection')

        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('stability_frames', 5)
        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('grasp_cooldown', 5.0)

        self._target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self._min_conf     = self.get_parameter('min_confidence').get_parameter_value().double_value
        self._stability    = self.get_parameter('stability_frames').get_parameter_value().integer_value
        self._pos_tol      = self.get_parameter('position_tolerance').get_parameter_value().double_value
        cooldown_sec       = self.get_parameter('grasp_cooldown').get_parameter_value().double_value

        self._cooldown = Duration(seconds=cooldown_sec)
        self._history = collections.deque(maxlen=self._stability)
        self._last_grasp_time = None

        self.create_subscription(Detection3DArray, '/detections_3d', self._cb, 10)
        self._pub = self.create_publisher(PoseStamped, '/grasp_target_pose', 10)

        self.get_logger().info(f"Watching for '{self._target_class}' (conf >= {self._min_conf})")

    def _cb(self, msg):
        best = None
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id != self._target_class:
                    continue
                if hyp.hypothesis.score < self._min_conf:
                    continue
                if best is None or hyp.hypothesis.score > best[0]:
                    best = (hyp.hypothesis.score, det)

        if best is None:
            self._history.clear()
            return

        pos = best[1].bbox.center.position
        self._history.append((pos.x, pos.y, pos.z))

        if len(self._history) < self._stability:
            return

        # check that all recent detections cluster within position tolerance
        xs = [p[0] for p in self._history]
        ys = [p[1] for p in self._history]
        spread = max(xs) - min(xs) + max(ys) - min(ys)
        if spread > self._pos_tol:
            return

        now = self.get_clock().now()
        if self._last_grasp_time is not None and (now - self._last_grasp_time) < self._cooldown:
            return

        self._last_grasp_time = now
        self._history.clear()

        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        mz = sum(p[2] for p in self._history) if self._history else pos.z

        target = PoseStamped()
        target.header.stamp = now.to_msg()
        target.header.frame_id = msg.header.frame_id
        target.pose.position.x = mx
        target.pose.position.y = my
        target.pose.position.z = mz
        target.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Stable '{self._target_class}' at ({mx:.2f}, {my:.2f}, {mz:.2f}) "
            f"[{msg.header.frame_id}] — publishing grasp target"
        )
        self._pub.publish(target)


def main():
    rclpy.init()
    rclpy.spin(GraspFromDetection())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
