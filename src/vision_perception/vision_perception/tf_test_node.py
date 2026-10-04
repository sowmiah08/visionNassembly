import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped


class TFTestNode(Node):

    def __init__(self):
        super().__init__('tf_test_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(
            1.0,
            self.test_transform
        )

    def test_transform(self):

        point_camera = PointStamped()

        # The point is expressed in the camera coordinate frame.
        point_camera.header.frame_id = 'camera_optical_frame'

        # A sample point 1 metre in front of the camera.
        point_camera.point.x = 0.0
        point_camera.point.y = 0.0
        point_camera.point.z = 1.0

        try:
            point_world = self.tf_buffer.transform(
                point_camera,
                'world',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'Camera point: '
                f'({point_camera.point.x:.3f}, '
                f'{point_camera.point.y:.3f}, '
                f'{point_camera.point.z:.3f}) '
                f'-> World point: '
                f'({point_world.point.x:.3f}, '
                f'{point_world.point.y:.3f}, '
                f'{point_world.point.z:.3f})'
            )

        except Exception as e:
            self.get_logger().warn(
                f'TF transform failed: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = TFTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()