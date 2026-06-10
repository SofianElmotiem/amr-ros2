#!/usr/bin/env python3
"""Convert /detections_3d to a PointCloud2 that Nav2 costmap layers can consume."""

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray

_POINT_STEP = 12  # 3 x float32


def _make_cloud(header, points):
    buf = bytearray(_POINT_STEP * len(points))
    for i, (x, y, z) in enumerate(points):
        struct.pack_into('<fff', buf, i * _POINT_STEP, float(x), float(y), float(z))
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * len(points)
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


class DetectionToCostmap(Node):

    def __init__(self):
        super().__init__('detection_to_costmap')
        self.create_subscription(Detection3DArray, '/detections_3d', self._cb, 10)
        self._pub = self.create_publisher(PointCloud2, '/detection_obstacles', 10)

    def _cb(self, msg):
        if not msg.detections:
            return

        all_points = []
        for det in msg.detections:
            if not det.results:
                continue
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            # footprint radius from detected bbox, with a minimum
            r = max(det.bbox.size.x, det.bbox.size.y) * 0.5
            r = max(r, 0.15)

            # ring of points at detection height + column down to ground
            angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
            for a in angles:
                px = x + r * np.cos(a)
                py = y + r * np.sin(a)
                all_points.append((px, py, z))
                all_points.append((px, py, 0.0))
            all_points.append((x, y, z))
            all_points.append((x, y, 0.0))

        if not all_points:
            return

        self._pub.publish(_make_cloud(msg.header, all_points))


def main():
    rclpy.init()
    rclpy.spin(DetectionToCostmap())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
