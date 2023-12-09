import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image # import image msg type from sensor_msgs


class MyDepthSub(Node):
    def __init__(self):
        super().__init__('my_depth_sub')

        # Note: it is very important to use a QoS profile for the subscriber that is compatible
        # with the QoS profile of the publisher.
        # The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
        # and durability set as "VOLATILE".
        # To be able to receive the subscribed topic the subscriber must use compatible parameters.
        depth_qos = rclpy.qos.QoSProfile(depth=10)
        depth_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        depth_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE        
        


        #create depth map subscriber
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered', #/zed/zed_node/depth/depth_registered
            self.depth_callback,
            depth_qos
        )


    def depth_callback(self, msg):
        # Get a pointer to the depth values casting the data pointer to floating point
        depths = memoryview(msg.data).cast('f')

        # Image coordinates of the center pixel
        u = msg.width // 2
        v = msg.height // 2

        #linear index of the center pixel
        center_idx = u + msg.width * v

        #output
        self.get_logger().info(f"Center Depth: {depths[center_idx]}")



def main(args=None):
    rclpy.init(args=args)

    depth_node = MyDepthSub()

    rclpy.spin(depth_node)

    depth_node.destroy_node()
    rclpy.shutdown()        

if __name__ == "__main__":
    main()