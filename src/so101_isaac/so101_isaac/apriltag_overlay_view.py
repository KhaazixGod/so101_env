"""One window: the camera's rectified feed with each detected tag's outline,
ID and camera-to-tag distance drawn directly on top.

apriltag_ros itself publishes no annotated image (just /apriltag/detections +
/tf), so this node builds the overlay itself: it keeps the latest
AprilTagDetectionArray, and on every new frame on /camera/color/image_rect it
draws each detection's 4 corners, looks up its /tf pose to compute distance
(same straight-line-norm approach as apriltag_distance_monitor.py), and shows
the annotated frame in an OpenCV window via cv2.imshow.
"""

import math

import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

_CORNER_COLOR = (0, 255, 0)
_TEXT_COLOR = (0, 255, 255)
_WINDOW = "AprilTag"


class AprilTagOverlayView(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_overlay_view")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("image_topic", "/camera/color/image_rect")
        self.declare_parameter("detections_topic", "/apriltag/detections")
        self.camera_frame = self.get_parameter("camera_frame").value

        self._bridge = CvBridge()
        self._latest_detections = AprilTagDetectionArray()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._on_image, 10,
        )

        cv2.namedWindow(_WINDOW, cv2.WINDOW_NORMAL)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_detections = msg

    def _tag_distance_m(self, tag_frame: str) -> float | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.camera_frame, tag_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        for det in self._latest_detections.detections:
            corners = [(int(c.x), int(c.y)) for c in det.corners]
            for i in range(4):
                cv2.line(frame, corners[i], corners[(i + 1) % 4], _CORNER_COLOR, 2)

            tag_frame = f"{det.family}:{det.id}"
            distance = self._tag_distance_m(tag_frame)
            label = f"id={det.id}"
            if distance is not None:
                label += f" {distance:.3f}m"

            cx, cy = int(det.centre.x), int(det.centre.y)
            cv2.putText(
                frame, label, (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, _TEXT_COLOR, 2, cv2.LINE_AA,
            )

        cv2.imshow(_WINDOW, frame)
        cv2.waitKey(1)


def main() -> None:
    rclpy.init()
    node = AprilTagOverlayView()
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
