#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, ObjectHypothesis
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()


class Detection3DNode(Node):

    def __init__(self):
        super().__init__('detection_3d')

        self.declare_parameter('target_frame', 'map')
        self._target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self._camera_info = None
        self._latest_depth = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth/image_raw', self._depth_cb, 10)
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self._inference_cb, 10)

        self._pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def _depth_cb(self, msg):
        self._latest_depth = msg

    def _inference_cb(self, msg):
        if self._camera_info is None or self._latest_depth is None:
            return

        depth_img = bridge.imgmsg_to_cv2(self._latest_depth, desired_encoding='passthrough')
        K = self._camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]
        cam_frame = self._latest_depth.header.frame_id

        # look up the latest available transform once for the whole batch
        # (using Time() = latest avoids "extrapolation into the future" errors,
        # since the inference header stamp lags behind the TF buffer)
        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, cam_frame, Time(), Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF {cam_frame} -> {self._target_frame} unavailable ({e}), '
                'publishing in camera frame',
                throttle_duration_sec=5.0,
            )

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._target_frame if transform is not None else cam_frame

        for det in msg.yolov8_inference:
            # InferenceResult xyxy: top=x1, left=y1, bottom=x2, right=y2
            u = (det.top + det.bottom) // 2   # pixel column
            v = (det.left + det.right) // 2   # pixel row

            h, w = depth_img.shape[:2]
            if not (0 <= v < h and 0 <= u < w):
                continue

            r = 3
            patch = depth_img[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].astype(np.float32)
            valid = patch[(patch > 0.05) & np.isfinite(patch)]
            if valid.size == 0:
                continue
            Z_cam = float(np.median(valid))

            X_cam = (u - cx) * Z_cam / fx
            Y_cam = (v - cy) * Z_cam / fy

            width_px = abs(det.bottom - det.top)
            height_px = abs(det.right - det.left)

            if transform is not None:
                cam_pose = Pose()
                cam_pose.position.x = X_cam
                cam_pose.position.y = Y_cam
                cam_pose.position.z = Z_cam
                cam_pose.orientation.w = 1.0
                map_pose = do_transform_pose(cam_pose, transform)
                X, Y, Z = map_pose.position.x, map_pose.position.y, map_pose.position.z
            else:
                X, Y, Z = X_cam, Y_cam, Z_cam

            d3 = Detection3D()
            d3.header = out.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis()
            hyp.hypothesis.class_id = det.class_name
            hyp.hypothesis.score = float(det.confidence)
            hyp.pose.pose.position = Point(x=X, y=Y, z=Z)
            hyp.pose.pose.orientation = Quaternion(w=1.0)
            d3.results.append(hyp)

            d3.id = str(det.track_id) if det.track_id >= 0 else ''
            d3.bbox.center.position = Point(x=X, y=Y, z=Z)
            d3.bbox.center.orientation = Quaternion(w=1.0)
            d3.bbox.size = Vector3(
                x=width_px * Z_cam / fx,
                y=height_px * Z_cam / fy,
                z=0.0,
            )

            out.detections.append(d3)

        self._pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(Detection3DNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
