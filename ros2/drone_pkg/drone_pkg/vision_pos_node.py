import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

sub_qos = rclpy.qos.QoSProfile(
    depth=20, 
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL.VOLATILE,  
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
)

class VisionPoseNode(Node):
    def __init__(self):
        super().__init__("vision_pose_node")
        # send both PoseWithCovarianceStamped and PoseStamped just in case
        
        self.vision_pose_cov_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic="/zed/zed_node/pose_with_covariance",
            callback=self.vision_pose_cov_callback,
            qos_profile=sub_qos
            )

        self.vision_pose_cov_pub = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic="/mavros/vision_pose/pose_cov",
            qos_profile=sub_qos,
            )

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic="/zed/zed_node/pose",
            callback=self.vision_pose_callback,
            qos_profile=sub_qos
            )

        self.vision_pose_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic="/mavros/vision_pose/pose",
            qos_profile=sub_qos,
            )

    def vision_pose_cov_callback(self, msg):
        self.vision_pose_cov_pub.publish(msg)
        print(msg)
    
    def vision_pose_callback(self, msg):
        self.vision_pose_pub.publish(msg)
        print(msg)


def main():
    rclpy.init()
    vision_pose_node = VisionPoseNode()
    rclpy.spin(vision_pose_node)
    rclpy.shutdown()