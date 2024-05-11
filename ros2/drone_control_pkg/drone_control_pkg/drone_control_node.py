
import rclpy 
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from std_msgs.msg import String


# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong

import transforms3d as tf3d
import math


# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

TARGET_QOS = rclpy.qos.QoSProfile(
    depth=20, 
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,  
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
)


# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

# PARAMETERS_QOS used for parameter streams
PARAMETERS_QOS = rclpy.qos.qos_profile_parameters



# gz sim -v4 -r iris_runway.sdf
# sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
# ros2 launch mavros px4.launch fcu_url:=tcp://127.0.0.1:5762@
# ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera model>

# in ardupilot sitl use: param set SIM_GPS_TYPE 0 and param set SIM_GPS2_TYPE 0 to disable gps


class DroneControllerNode(Node):

    def __init__(self):
        super().__init__('drone_controller_node')
 
        self.last_state = None # last status message
        self.local_pos = None # last known local position message
        self.last_target_msg = PoseStamped()  # last sent target position message
        
        self.landing_target_position = None # last known landing target position message
        self.landing_target_drone_position = None # drones position when the last landing target position message was received

        self.avocado_target_position = None # last known avocado target position message
        self.avocado_target_drone_position = None # drones position when the last avocado target position message was received
        
        self.current_detection_target = 'landing_pad' # used to tell the trt_node to which object to search for. can be 'landing_pad' or 'avocado' default is landing pad
        
        self.is_landing_with_vision = False
        
        # create service clients
        # for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command service not available, waiting again...')

        # for mode changes ...
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')

        # for arming ...
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')

        # for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        #publisher for setpoint
        self.target_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        # publisher for the current detection target variable
        self.current_detection_target_pub = self.create_publisher(String, '/my_drone/current_detection_target', 10) 
    
        # sub for vehicle state
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, STATE_QOS)
        # sub for local position
        self.pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_position_callback, SENSOR_QOS)
        # sub for target landing position x y and z are distance from the drone to the target in meters
        self.landing_target_sub = self.create_subscription(PointStamped, '/my_drone/landing_target_position', self.landing_target_callback, SENSOR_QOS)
        # sub for avocado target position x y and z are distance from the drone to the target in meters
        self.avocado_target_sub = self.create_subscription(PointStamped, '/my_drone/avocado_target_position', self.avocado_target_callback, SENSOR_QOS)
        
        # publisher to send landing_target messages to mavros
        self.landing_target_pub = self.create_publisher(PoseStamped, '/mavros/landing_target/pose_in', qos_profile=TARGET_QOS )
        
        self.start()

    def avocado_target_callback(self, msg):
        # save the last avocado target position message
        self.avocado_target_position = msg
        # save the drones position when the last avocado target position message was received
        self.avocado_target_drone_position = self.local_pos
        self.get_logger().debug('Avocado target position: {}'.format(msg.point))
    
    def landing_target_callback(self, msg):
        # save the last landing target position message
        self.landing_target_position = msg
        # save the drones position when the last landing target position message was received
        self.landing_target_drone_position = self.local_pos
        self.get_logger().debug('Landing target position: {}'.format(msg.point))          
   
    def request_data_stream(self, msg_id, msg_interval):
        # request data streams from the vehicle. msg_id is the MAVLink message ID, msg_interval is the interval in microseconds
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future) # wait for response))

    def wait_for_new_status(self):
            """
            Wait for new state message to be received.  These are sent at
            1Hz so calling this is roughly equivalent to one second delay.
            """
            if self.last_state:
                # if had a message before, wait for higher timestamp
                last_stamp = self.last_state.header.stamp.sec
                for _ in range(60):
                    rclpy.spin_once(self)
                    if self.last_state.header.stamp.sec > last_stamp:
                        break
            else:
                # if never had a message, just wait for first one          
                for _ in range(60):
                    if self.last_state:
                        break
                    rclpy.spin_once(self)

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
        
    # save the last status message.nanoseconds / 1e9
    def state_callback(self, msg):
        self.last_state = msg
        
        self.get_logger().debug('Mode: {}. Armed: {}'.format(msg.mode, msg.armed))
        

    # save the last local position message gets the rotation as well
    def local_position_callback(self, msg):
        self.local_pos = msg 
        
        self.get_logger().debug('Local position: {}'.format(msg.pose.position))     
        
    # send a setpoint message to move the vehicle to a local position
    def move_local(self, x:float, y:float, z:float, wait_for_pos: bool = True, error_tolerance: float=0.1):
        """
        Move the vehicle to a local position. keeping the current rotation.
        """ 
        
        try:
            x, y, z = float(x), float(y), float(z)
        
        except Exception as e:
            self.get_logger().error(e)
        
        
        # set the header
        self.last_target_msg.header.stamp = self.get_clock().now().to_msg()
        # set the position
        self.last_target_msg.pose.position.x = x
        self.last_target_msg.pose.position.y = y
        self.last_target_msg.pose.position.z = z

        # keep the current rotation
        self.last_target_msg.pose.orientation.w = self.local_pos.pose.orientation.w
        self.last_target_msg.pose.orientation.x = self.local_pos.pose.orientation.x
        self.last_target_msg.pose.orientation.y = self.local_pos.pose.orientation.y
        self.last_target_msg.pose.orientation.z = self.local_pos.pose.orientation.z

        # publish the message
        self.target_pub.publish(self.last_target_msg)
        self.get_logger().info('Moving to {} {} {}'.format(x,y,z))
        
        if wait_for_pos:
            self.wait_until_pos_reached(x, y, z, error_tolerance=error_tolerance)
        


    def wait_until_pos_reached(self, target_x = 0.0, target_y = 0.0, target_z = 0.0, check_last_target_pose = True, error_tolerance=0.5, timeout=10):
        """
        Wait until the last setpoint message is reached.
        Only use target_x, target_y, target_z if check_last_target_pose is false. used for the takeoff and other stuff that does not use the last target pose.
        """
        
        # for timout seconds, in a loop wait for a new local position message 
        # then check if the position of the drone is within the error tolerance of the target position
        # if so, return.  Otherwise, keep waiting.

        # target position when the last setpoint message was sent. if false use the user given target position.
        if check_last_target_pose:
            target_x = self.last_target_msg.pose.position.x
            target_y = self.last_target_msg.pose.position.y
            target_z = self.last_target_msg.pose.position.z

        start_time = self.get_clock().now() # start of the waiting period
        last_print_time  = start_time 
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout: # wait for timeout seconds
            rclpy.spin_once(self) 

            # wait for a new local position message
            if self.local_pos: # if there was a message before, wait for a higher timestamp
                last_stamp = self.local_pos.header.stamp.sec
                for _ in range(5): # wait for 5 seconds
                    rclpy.spin_once(self)
                    if self.local_pos.header.stamp.sec > last_stamp:
                        break
                    
            else: # if there was no message before, just wait for the first one
                for _ in range(5):
                    if self.local_pos:
                        break
                    rclpy.spin_once(self)
        
            # check if the position of the drone is within the error tolerance of the target position
            dx = abs(target_x - self.local_pos.pose.position.x)
            dy = abs(target_y - self.local_pos.pose.position.y)
            dz = abs(target_z - self.local_pos.pose.position.z)
               
            # difference in 3d space
            dist = (dx**2 + dy**2 + dz**2)**0.5

            if dist <= error_tolerance:
                self.get_logger().info('Target position reached. x:{} y:{} z:{} dist:{}'.format(target_x, target_y, target_z, dist))
                return True # target position reached
            
            else:
                # print time left until timeout. only print every second      
                if (self.get_clock().now() - last_print_time).nanoseconds / 1e9 > 1:               
                    self.get_logger().debug('Waiting for target position. x:{} y:{} z:{} dist:{} timeout:{}'.format(target_x, target_y, target_z, dist, timeout - (self.get_clock().now() - start_time).nanoseconds / 1e9))
                    last_print_time = self.get_clock().now()


        # timeout reached
        self.get_logger().debug('Timeout reached. could not reach target position. x:{} y:{} z:{} dist:{} '.format(target_x, target_y, target_z, dist))
        return False



    def rotate_to(self, radians):
        """
        Rotate the drone to the given radians. 
        """
        
        # set the header
        self.last_target_msg.header.stamp = self.get_clock().now().to_msg()
        # position of the drone will be the same
        self.last_target_msg.pose.position.x = self.local_pos.pose.position.x
        self.last_target_msg.pose.position.y = self.local_pos.pose.position.y
        self.last_target_msg.pose.position.z = self.local_pos.pose.position.z

        # calculate the rotation quaternion
        q = tf3d.euler.euler2quat(0.0, 0.0, radians)
        self.last_target_msg.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])

        # publish the message
        self.target_pub.publish(self.last_target_msg)


    def wait_for(self, seconds):
        """
        Wait for the given number of seconds. 
        """
        
        self.get_logger().debug('Waiting for {} seconds'.format(seconds))
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self)   
        return True    

    
    def do_360(self, step_size = 60.0, step_time = 5):
        """
        rotate the drone 360 degrees. 
        """
        
        # convert to radians
        step_size_radians = math.radians(step_size) 
        
        # get the current yaw in radians
        current_yaw_radians = tf3d.euler.quat2euler((self.local_pos.pose.orientation.w, self.local_pos.pose.orientation.x, 
                                                     self.local_pos.pose.orientation.y, self.local_pos.pose.orientation.z))[2] 

        # calculate the number of steps needed to complete a 360 degree rotation
        total_steps = int(2 * math.pi / step_size_radians)
        rotated = current_yaw_radians

        for i in range(total_steps + 1):
            # calculate the target yaw
            target_yaw_radians = (rotated + step_size_radians) % (2 * math.pi)
            # convert to quaternion
            target_q = tf3d.euler.euler2quat(0.0, 0.0, target_yaw_radians)
            # rotate using rotate_to function
            self.rotate_to(target_yaw_radians)
            # wait for the rotation to complete
            rotated += step_size_radians
            self.wait_for(step_time)

        # rotate to the original yaw just in case the drone rotated more than 360 degrees
        self.rotate_to(current_yaw_radians)    
        self.wait_for(step_time)

    def send_landing_target(self, landing_pos):
        """Sends a landing_target message to mavros assumes position is in local ned frame.
        Args:
            landing_pos: x y z of the landing_target.
        """
        msg_to_send = PoseStamped()
        msg_to_send.header.stamp = self.get_clock().now().to_msg()
        msg_to_send.pose.position.x = landing_pos['x'] 
        msg_to_send.pose.position.y = landing_pos['y']
        msg_to_send.pose.position.z = landing_pos['z']

        self.landing_target_pub.publish(msg_to_send)
        print("sent vision landing message")

    def start(self):
        
        # send current detection target to the trt_node. which is landing_pad at start
        #msg = String()
        #msg.data = self.current_detection_target
        #self.get_logger().info('Setting current detection target to {}'.format(msg.data))
        #self.current_detection_target_pub.publish(msg)


        # some vars to be used during flight
        takeoff_altitude = 1.0
        descend_rate = 0.1 # meters
        descend_threshold = 1.0 # meters


        # wait until system status becomes standby
        for _ in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status == 3:
                self.get_logger().info('System status: Standby')
                break

        # request data streams
        self.request_data_stream(32, 100000) # local position
        self.get_logger().info('Requested local position stream')

        self.request_data_stream(31, 100000) # attitude

        # change mode to GUIDED
        #self.change_mode('GUIDED')
        #self.get_logger().info('Requested GUIDED mode')

        mode_req = SetMode.Request()
        # https://mavlink.io/en/messages/common.html#MAV_MODE_GUIDED_DISARMED
        mode_req.base_mode = 88 
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future) 
    
        # try to arm the drone.
        for _ in range(60):
            self.arm_request()
            self.get_logger().info('Arming request sent.')
            self.wait_for_new_status()
            if self.last_state.armed:
                self.get_logger().info('Arming successful')
                break
        else:
            self.get_logger().error('Failed to arm')
        
        
        ##################################
        #          start mission         #
        ##################################    

        # take off and climb to 3 meters at current location
        self.takeoff(takeoff_altitude)
        self.get_logger().info('Takeoff request sent.')
        self.wait_for(60)
        # wait for drone to reach desired altitude
        #if self.wait_until_pos_reached(target_x=self.local_pos.pose.position.x, target_y=self.local_pos.pose.position.y, target_z=takeoff_altitude, check_last_target_pose=False):
        #    self.get_logger().info('Reached target altitude')
        #else:
        #    self.get_logger().error('Failed to reach target altitude') 
        
    
    
        self.change_mode('LAND')
        # ------- Takeoff end ------- #

        return
        # positions the drone will move to.
        # NOTE: Implement a way to load mission from file
        #self.move_local(1, 0, 1)
        #self.move_local(0, 1, 1)
        #self.move_local(-1, 0, 1)
        #self.move_local(0, -1, 1)
        #self.move_local(1, 0, 1)
        #self.move_local(0, 0, 1)
        
        # ------- landing start ------- #
       
        # if landing pad was never found land using LAND mode
        if self.landing_target_position is None: # or self.landing_target_drone_position is None:
            self.get_logger().warning('No landing target position received. Landing using RTL mode')
            self.change_mode('LAND')
            self.get_logger().info('Requested LAND mode')
        
        # if position was found start vision landing
        else:
            self.get_logger().info("Vision landing started")
            self.is_landing_with_vision = True
        
                
        while self.is_landing_with_vision:
            pos_to_send = {'x' : self.landing_target_position.pose.position.x, 
                           'y' : self.landing_target_position.pose.position.y,
                           'z' : self.landing_target_position.pose.position.z}
            
            self.send_landing_target(pos_to_send)
            rclpy.spin_once(self)

        
        ##################################
        #         Mission end            #
        ##################################
        
        while rclpy.ok():
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    drone_controller_node = DroneControllerNode()
    rclpy.spin(drone_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
