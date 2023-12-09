import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Message Received: {msg.data}')


def main():
    rclpy.init()

    sub = TestSubscriber()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()




