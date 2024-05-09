import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, SetMode

from geometry_msgs.msg import PoseStamped

import numpy as np

# QoS profile used for the state subscriber topics.
STATE_QOS = rclpy.qos.QoSProfile(
    depth=10,
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_ALL,
)

# QoS profile used for the pose subscriber topics.
POSE_QOS = rclpy.qos.QoSProfile(
    depth=10,
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
)

class DroneSetup(Node):
    def __init__(self):
        super().__init__('drone_setup')
        
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        
        # service to send commands to the drone. Mainly used for requesting datastream requests to access the drone's data from mavros topics.
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command service not available, waiting again...')

        # request a periodic messages from the fcu. We do this because sometimes fcu does not send needed messages to mavros.
        self.set_message_interval(32, 100000) # local position
        self.set_message_interval(30, 100000) # attitude
        self.set_message_interval(26, 100000) # scaled imu
        self.set_message_interval(27, 100000) # raw imu
        
        # set mode to guided 
        mode_req = SetMode.Request()
        # https://mavlink.io/en/messages/common.html#MAV_MODE_GUIDED_DISARMED
        mode_req.base_mode = 88 
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future) 
        
    def set_message_interval(self, msg_id: int, msg_interval: int):
        """ Request a periodic message from the fcu. 
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


def main(args=None):
    rclpy.init(args=args)
    node = DroneSetup()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()