"""Camera viewer + AprilTag detection, using pupil-apriltags (NOT ROS's
apriltag/apriltag_ros -- a plain pip package, no ROS dependency at all for the
detection step itself):

    pip install --user --break-system-packages pupil-apriltags

Subscribes to astra_camera's raw colour image + camera_info, detects tags with
pupil_apriltags.Detector (estimate_tag_pose=True gives translation/rotation
directly -- no manual solvePnP needed, unlike apriltag_custom_detector.py's
approach), draws each tag's outline/ID/distance on the frame, and shows it in
a window. Also logs/publishes distance on /apriltag/distances, same format as
apriltag_distance_monitor.py, so that still works for a quick terminal check.

    ros2 run so101_isaac camera_viewer
    ros2 run so101_isaac camera_viewer --ros-args -p image_topic:=/camera/depth/image_raw
      -> falls back to plain viewing (no camera_info/tag size makes sense for
         a depth stream), detection simply reports nothing

Tag family/size are node parameters here, not config/apriltag.yaml (that file
is apriltag_ros's own config format; this node has nothing to do with it).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from pupil_apriltags import Detector

_WINDOW = "Camera"
_CORNER_COLOR = (0, 255, 0)
_TEXT_COLOR = (0, 255, 255)


class CameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("camera_viewer")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("family", "tag36h11")
        self.declare_parameter("size", 0.04)
        self.declare_parameter("max_hamming", 0)
        self.declare_parameter("threads", 8)
        self.declare_parameter("decimate", 1.0)

        self._size = float(self.get_parameter("size").value)
        self._max_hamming = int(self.get_parameter("max_hamming").value)
        self._camera_info: CameraInfo | None = None
        self._bridge = CvBridge()

        self._detector = Detector(
            families=self.get_parameter("family").value,
            nthreads=int(self.get_parameter("threads").value),
            quad_decimate=float(self.get_parameter("decimate").value),
        )

        image_topic = self.get_parameter("image_topic").value
        self._pub = self.create_publisher(String, "/apriltag/distances", 10)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._on_camera_info, 10,
        )
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f"showing {image_topic}")

        cv2.namedWindow(_WINDOW, cv2.WINDOW_NORMAL)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        encoding = "bgr8" if msg.encoding in ("bgr8", "rgb8") else "passthrough"
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)

        if self._camera_info is not None and frame.ndim == 3:
            self._detect_and_draw(frame)

        cv2.imshow(_WINDOW, frame)
        cv2.waitKey(1)

    def _detect_and_draw(self, frame: np.ndarray) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        k = self._camera_info.k
        camera_params = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

        detections = self._detector.detect(
            gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=self._size,
        )

        parts = []
        for det in detections:
            if det.hamming > self._max_hamming:
                continue

            corners = det.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), _CORNER_COLOR, 2)

            distance = float(np.linalg.norm(det.pose_t))
            label = f"id={det.tag_id} {distance:.3f}m"
            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.putText(
                frame, label, (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, _TEXT_COLOR, 2, cv2.LINE_AA,
            )
            parts.append(f"{det.tag_family.decode()}:{det.tag_id}={distance:.3f}m")

        self._pub.publish(String(data=" ".join(parts)))


def main() -> None:
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
