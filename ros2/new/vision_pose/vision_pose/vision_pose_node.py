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

        self.last_pose = PoseStamped()
        
        
    def vision_pose_callback(self, msg:PoseStamped):
        self.last_pose = msg
        self.vision_pose_pub.publish(self.last_pose)

def main():
    rclpy.init()
    vision_pose_node = VisionPoseNode()
    rclpy.spin(vision_pose_node)
    rclpy.shutdown()