import rclpy
from rclpy.node import Node
from rclpy import qos

from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandLong, SetMode

from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import Point, Quaternion, Vector3

# QoS profile used for the state subscriber topics.
STATE_QOS = qos.QoSProfile(
    depth=10,
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_ALL,
)

# QoS profile used for the pose subscriber topics.
PUB_QOS = qos.QoSProfile(
    depth=10,
    durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=qos.QoSReliabilityPolicy.RELIABLE,
)


class DroneSetup(Node):
    def __init__(self):
        super().__init__("drone_setup")

        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        # service to send commands to the drone. Mainly used for requesting datastream
        # requests to access the drone's data from mavros topics.
        self.cmd_cli = self.create_client(CommandLong, "/mavros/cmd/command")

        # publisher to set HOME_POSITION
        self.set_home_pub = self.create_publisher(
            HomePosition, "/mavros/home_position/set", PUB_QOS
        )

        # publisher to set GPS_GLOBAL_ORIGIN
        # https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN
        self.set_gp_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", PUB_QOS
        )

        self.set_gp_origin()
        self.set_home_pos()

        # request a periodic messages from the fcu. We do this because sometimes fcu does not send needed messages to mavros.
        self.set_message_interval(32, 100000)  # local position
        self.set_message_interval(30, 100000)  # attitude
        self.set_message_interval(26, 100000)  # scaled imu
        self.set_message_interval(27, 100000)  # raw imu

        # set mode to guided
        mode_req = SetMode.Request()
        # https://mavlink.io/en/messages/common.html#MAV_MODE_GUIDED_DISARMED
        mode_req.base_mode = 88
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)

    def set_message_interval(self, msg_id: int, msg_interval: int):
        """Request a periodic message from the fcu.
            For message ids: https://mavlink.io/en/messages/common.html

        Args:
            msg_id (int): Mavlink message id of the requested message.
            msg_interval (float): Interval in microseconds between two messages. 0 to request default interval.
        """

        cmd_req = CommandLong.Request()
        # mavlink message id of the MAV_CMD_SET_MESSAGE_INTERVAL message which we use to request a message.
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)

    def set_home_pos(self):
        self.get_logger().info("Setting home position.")
        msg = HomePosition()

        msg.header.stamp = self.get_clock().now().to_msg()

        # Geo point
        msg.geo.altitude = 0.0
        msg.geo.latitude = 0.0
        msg.geo.longitude = 0.0

        # position
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0

        # orientation
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # approach. Vector3
        msg.approach.x = 0.0
        msg.approach.y = 0.0
        msg.approach.z = 1.0

        self.set_home_pub.publish(msg)

    def set_gp_origin(self):
        self.get_logger().info("Setting global position origin")
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position.altitude = 0.0
        msg.position.latitude = 0.0
        msg.position.longitude = 0.0

        self.set_gp_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSetup()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()