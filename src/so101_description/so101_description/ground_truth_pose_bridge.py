#!/usr/bin/env python3
"""Publish the true world poses of free objects (plug, peg, ...) from Gazebo
as /ground_truth/<name>_pose. For evaluation only: perception must never
read these topics.

The standard bridge can't do this (its converted poses have no object
names), so this node reads Gazebo's /world/<world>/dynamic_pose/info
directly and republishes each tracked model by name.
"""

import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import PoseStamped

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


class GroundTruthPoseBridge(RosNode):

    def __init__(self):
        super().__init__('ground_truth_pose_bridge')

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('tracked_models', ['plug'])

        world_name = self.get_parameter('world_name').value
        tracked_models = self.get_parameter('tracked_models').value

        self._publishers = {
            name: self.create_publisher(PoseStamped, f'/ground_truth/{name}_pose', 10)
            for name in tracked_models
        }

        self._gz_node = GzNode()
        topic = f'/world/{world_name}/dynamic_pose/info'
        if not self._gz_node.subscribe(Pose_V, topic, self._on_pose_v):
            self.get_logger().error(f'Failed to subscribe to Gazebo topic {topic}')
        else:
            self.get_logger().info(
                f'Bridging ground-truth poses for {tracked_models} from {topic}'
            )

    def _on_pose_v(self, msg: Pose_V):
        stamp = self.get_clock().now().to_msg()
        for pose in msg.pose:
            publisher = self._publishers.get(pose.name)
            if publisher is None:
                continue
            out = PoseStamped()
            out.header.stamp = stamp
            out.header.frame_id = 'world'
            out.pose.position.x = pose.position.x
            out.pose.position.y = pose.position.y
            out.pose.position.z = pose.position.z
            out.pose.orientation.x = pose.orientation.x
            out.pose.orientation.y = pose.orientation.y
            out.pose.orientation.z = pose.orientation.z
            out.pose.orientation.w = pose.orientation.w
            publisher.publish(out)


def main():
    rclpy.init()
    node = GroundTruthPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
