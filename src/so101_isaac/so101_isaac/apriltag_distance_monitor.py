"""Reports the camera-to-tag distance for every AprilTag currently detected.

apriltag_ros's AprilTagDetection message carries no pose field -- with
pose_estimation_method set, it instead broadcasts each tag's pose on /tf as
"<family>:<id>" (e.g. "tag36h11:0") relative to the camera's optical frame.
This node watches /apriltag/detections for which tags are currently visible,
looks up each one's /tf transform, and reports straight-line distance
(the transform's translation norm) in metres:

- logged to the console (throttled, one line per tag)
- published on /apriltag/distances as a human-readable std_msgs/String,
  e.g. "tag36h11:0=0.321m tag36h11:3=0.550m" (empty string when no tag is
  in view) -- `ros2 topic echo /apriltag/distances` to watch it live.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AprilTagDistanceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_distance_monitor")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("detections_topic", "/apriltag/detections")
        self.declare_parameter("distances_topic", "/apriltag/distances")
        self.camera_frame = self.get_parameter("camera_frame").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(String, self.get_parameter("distances_topic").value, 10)
        self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10,
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        parts = []
        for det in msg.detections:
            tag_frame = f"{det.family}:{det.id}"
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.camera_frame, tag_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"no /tf for {tag_frame} yet: {exc}", throttle_duration_sec=2.0,
                )
                continue
            t = tf.transform.translation
            distance = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
            parts.append(f"{tag_frame}={distance:.3f}m")

        line = " ".join(parts)
        self._pub.publish(String(data=line))
        if parts:
            self.get_logger().info(line, throttle_duration_sec=1.0)


def main() -> None:
    rclpy.init()
    node = AprilTagDistanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
