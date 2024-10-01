import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_sensor_data
import rclpy.qos


class VisionPoseNode(Node):
    def __init__(self):
        super().__init__("vision_pose_node")

        # QoS profile of the publisher
        pub_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Local pose estimate from the ekf filter. Fused VIO + IMU from the zed camera.
        self.vision_pose_sub = self.create_subscription(
            msg_type=Odometry,
            topic="/ekf/local",
            callback=self.vision_pose_callback,
            qos_profile=qos_profile_sensor_data,
        )

        # Publisher for the local pose
        self.vision_pose_pub = self.create_publisher(
            msg_type=PoseStamped, topic="/mavros/vision_pose/pose", qos_profile=pub_qos
        )

    def vision_pose_callback(self, msg: Odometry):
        """
        Callback for the local pose estimate from the ekf filter.
        Args:
            msg (Odometry): Odometry message received from the ekf filter.

        Returns:

        """
        pub_msg = PoseStamped()
        pub_msg.header.stamp = msg.header.stamp
        pub_msg.header.frame_id = msg.header.frame_id
        pub_msg.pose.position.x = msg.pose.pose.position.x
        pub_msg.pose.position.y = msg.pose.pose.position.y
        pub_msg.pose.position.z = msg.pose.pose.position.z

        pub_msg.pose.orientation.x = msg.pose.pose.orientation.x
        pub_msg.pose.orientation.y = msg.pose.pose.orientation.y
        pub_msg.pose.orientation.z = msg.pose.pose.orientation.z
        pub_msg.pose.orientation.w = msg.pose.pose.orientation.w

        self.vision_pose_pub.publish(pub_msg)


def main():
    rclpy.init()
    vision_pose_node = VisionPoseNode()
    rclpy.spin(vision_pose_node)
    rclpy.shutdown()