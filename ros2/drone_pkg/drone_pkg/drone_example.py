"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State                               # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped                  # type: ignore

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore

# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

# PARAMETERS_QOS used for parameter streams
PARAMETERS_QOS = rclpy.qos.qos_profile_parameters

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('modular_controller')
        self.last_state = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start
        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
        
    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_state = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))

    def wait_for_new_status(self):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.
        """
        if self.last_state:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_state.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_state.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_state:
                    break
                rclpy.spin_once(self)

    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

    def run(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, STATE_QOS)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.position_callback, SENSOR_QOS)

        # first wait for the system status to become 3 "standby"
        # see https://mavlink.io/en/messages/common.html#MAV_STATE
        for try_standby in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status==3:
                self.get_logger().info('Drone ready for flight')
                break 

        # send command to request regular position updates
        self.request_data_stream(33, 1000000)
        self.get_logger().info('Requested position stream')

        # now change mode to GUIDED
        self.change_mode("GUIDED")
        self.get_logger().info('Request sent for GUIDED mode.')
        
        # next, try to arm the drone
        # keep trying until arming detected in state message, or 60 attempts
        for try_arm in range(60):
            self.arm_request()
            self.get_logger().info('Arming request sent.')
            self.wait_for_new_status()
            if self.last_state.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.last_pos:
                    self.init_alt = self.last_pos.altitude
                break
        else:
            self.get_logger().error('Failed to arm')

        # take off and climb to 20.0m at current location
        self.takeoff(20.0)
        self.get_logger().info('Takeoff request sent.')

        # wait for drone to reach desired altitude, or 60 attempts
        for try_alt in range(60):
            self.wait_for_new_status()
            self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                break

        # move drone by sending setpoint message
        self.flyto(51.423, -2.671, self.init_alt - 30.0) # unexplained correction factor on altitude

        # wait for drone to reach desired position, or timeout after 60 attempts
        for try_arrive in range(60):
            self.wait_for_new_status()
            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if abs(d_lon) < 0.0001:
                if abs(d_lat) < 0.0001:
                    self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                    break

        # return home and land
        self.change_mode("RTL")
        self.get_logger().info('Request sent for RTL mode.')
        
        # now just serve out the time until process killed
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()


if __name__ == '__main__':
    main()