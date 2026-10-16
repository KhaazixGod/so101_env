"""Custom AprilTag detector -- drop-in replacement for apriltag_ros + image_proc::RectifyNode.

Subscribes directly to astra_camera's raw colour stream and camera_info (no
separate rectify node: this camera's distortion coefficients are all zero, so
rectification would be a no-op anyway; cv2.undistort() is still applied when
non-zero distortion is present, in case that ever changes). Detection uses the
`apriltag` Python bindings that ros-jazzy-apriltag already ships (the same
libapriltag C library apriltag_ros itself wraps -- this is not a different
detector, just a different, simpler process wrapping it directly instead of
going through apriltag_ros's ROS node). Pose (and therefore distance) comes
from cv2.solvePnP using the tag's 4 detected corners + known tag size,
not from apriltag_ros's internal pose estimator.

Publishes the exact same /apriltag/detections (apriltag_msgs/AprilTagDetectionArray)
and /tf ("<family>:<id>" frames) that apriltag_ros produced, so
apriltag_distance_monitor.py and apriltag_overlay_view.py work unmodified
against this node instead.
"""

import apriltag as apriltag_lib
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster


def _rotation_matrix_to_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return ((r[2, 1] - r[1, 2]) * s, (r[0, 2] - r[2, 0]) * s,
                (r[1, 0] - r[0, 1]) * s, 0.25 / s)
    i = np.argmax([r[0, 0], r[1, 1], r[2, 2]])
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                (r[0, 2] - r[2, 0]) / s)
    s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
            (r[1, 0] - r[0, 1]) / s)


class AprilTagCustomDetector(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_custom_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("detections_topic", "/apriltag/detections")
        # Dot-separated names so the same config/apriltag.yaml used by
        # apriltag_ros (which nests threads/decimate/etc. under "detector:")
        # works unchanged for this node too.
        self.declare_parameter("family", "36h11")
        self.declare_parameter("size", 0.04)
        self.declare_parameter("max_hamming", 0)
        self.declare_parameter("detector.threads", 8)
        self.declare_parameter("detector.decimate", 1.0)
        self.declare_parameter("detector.blur", 0.0)
        self.declare_parameter("detector.refine", True)

        # apriltag.yaml follows apriltag_ros's own convention of a bare
        # family name ("36h11"); this binding's constructor wants "tag36h11".
        family_param = self.get_parameter("family").value
        self._family = family_param if family_param.startswith("tag") else f"tag{family_param}"
        self._size = float(self.get_parameter("size").value)
        self._max_hamming = int(self.get_parameter("max_hamming").value)
        half = self._size / 2.0
        # Matches the "lb-rb-rt-lt" corner order the detector returns.
        self._object_points = np.array([
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ], dtype=np.float64)

        self._detector = apriltag_lib.apriltag(
            self._family,
            threads=int(self.get_parameter("detector.threads").value),
            decimate=float(self.get_parameter("detector.decimate").value),
            blur=float(self.get_parameter("detector.blur").value),
            refine_edges=bool(self.get_parameter("detector.refine").value),
        )

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._tf_broadcaster = TransformBroadcaster(self)
        self._pub = self.create_publisher(
            AprilTagDetectionArray, self.get_parameter("detections_topic").value, 10,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._on_camera_info, 10,
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._on_image, 10,
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        if self._camera_info is None:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        k = self._camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist = np.array(self._camera_info.d, dtype=np.float64)
        if dist.size and np.any(dist):
            gray = cv2.undistort(gray, camera_matrix, dist)

        out = AprilTagDetectionArray()
        out.header = msg.header

        for det in self._detector.detect(gray):
            if det["hamming"] > self._max_hamming:
                continue
            corners = np.array(det["lb-rb-rt-lt"], dtype=np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                self._object_points, corners, camera_matrix, np.zeros(5),
            )
            if not ok:
                continue

            out_det = AprilTagDetection()
            out_det.family = self._family
            out_det.id = int(det["id"])
            out_det.hamming = int(det["hamming"])
            out_det.decision_margin = float(det["margin"])
            out_det.goodness = float(det["margin"])
            out_det.centre = Point(x=float(det["center"][0]), y=float(det["center"][1]))
            out_det.corners = [Point(x=float(c[0]), y=float(c[1])) for c in corners]
            out.detections.append(out_det)

            self._broadcast_tf(msg.header, out_det.family, out_det.id, rvec, tvec)

        self._pub.publish(out)

    def _broadcast_tf(self, header, family: str, tag_id: int, rvec, tvec) -> None:
        rot, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rot)
        t = TransformStamped()
        t.header = header
        t.child_frame_id = f"{family}:{tag_id}"
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = AprilTagCustomDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
