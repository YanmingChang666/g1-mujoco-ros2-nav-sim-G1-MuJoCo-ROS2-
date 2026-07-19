#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class FastlioTfBridge(Node):
    def __init__(self):
        super().__init__("fastlio_tf_bridge")

        self.declare_parameter("fastlio_odom_topic", "/Odometry")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_map_to_odom", True)
        # FAST-LIO emits ~1 pose per LiDAR scan (10 Hz). Nav2's controller and
        # costmaps need a fresher odom->base_link than that, so the last pose
        # is re-broadcast with a current stamp at this rate. 0 disables.
        self.declare_parameter("tf_rate", 50.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_map_to_odom = bool(
            self.get_parameter("publish_map_to_odom").value
        )

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 20
        )
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.get_parameter("fastlio_odom_topic").value,
            self._odom_callback,
            20,
        )
        if self.publish_map_to_odom:
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            map_to_odom.transform.rotation.w = 1.0
            self.static_tf_pub.sendTransform(map_to_odom)

        self._last_pose = None
        self._last_twist = None
        tf_rate = float(self.get_parameter("tf_rate").value)
        if tf_rate > 0.0:
            self.create_timer(1.0 / tf_rate, self._rebroadcast_timer)

        self.get_logger().info(
            "Bridging FAST-LIO /Odometry to /odom and odom->base_link TF"
        )

    def _odom_callback(self, msg):
        self._last_pose = msg.pose
        self._last_twist = msg.twist
        self._publish(msg.header.stamp)

    def _rebroadcast_timer(self):
        if self._last_pose is None:
            return
        self._publish(self.get_clock().now().to_msg())

    def _publish(self, stamp):
        pose = self._last_pose

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = pose.pose.position.x
        odom_to_base.transform.translation.y = pose.pose.position.y
        # Nav2's base_link is a planar navigation frame. FAST-LIO estimates a
        # 3D body pose, but using that z here would double-count sensor height
        # with base_link->livox_frame and make the simulated Mid360 cloud float.
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation = pose.pose.orientation
        self.tf_pub.sendTransform(odom_to_base)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = pose
        out.pose.pose.position.z = 0.0
        out.twist = self._last_twist
        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FastlioTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
