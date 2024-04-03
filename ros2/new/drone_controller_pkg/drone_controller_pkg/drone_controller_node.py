
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State as DroneState
from mavros_msgs.srv import SetMode, CommandTOL, CommandBool, CommandLong

import numpy as np
from typing import List
from collections import deque
import transforms3d as tf3d


# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

TARGET_QOS = rclpy.qos.QoSProfile(
    depth=20, 
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL.VOLATILE,  
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
)

# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

# PARAMETERS_QOS used for parameter streams
PARAMETERS_QOS = rclpy.qos.qos_profile_parameters

class Pose3D:
    def __init__(self, pos: List[float], rot: List[float], init_time: float, frame_id: str='relative_frame'):
        """ a point in 3d space with rotation.
            Assumes the rotation is a quaternion, and the position is in meters.

        Args:
            pos (List[float], optional): Position of point in 3d space. Defaults to None.
            rot (List[float], optional): quaterion rotation of point in 3d space. Defaults to None.
            init_time (float, optional): time of initialization. Defaults to None.
            frame_id (str, optional): frame id of point. Defaults to "relative_frame".
        """
        
        if len(rot)!= 4:
            raise ValueError("Rotation must be a quaternion")
        
        if len(pos)!= 3:
            raise ValueError("Position must be a vector of length 3")
        
        self._frame_id = frame_id
        self._init_time = init_time
        self._rot = np.array(rot, dtype=np.float64) if rot else np.nan 
        self._pos = np.array(pos, dtype=np.float64)

    @property
    def pos(self) -> np.ndarray:
        return self._pos
    
    @property
    def rot(self) -> np.ndarray:
        return self._rot
    
    @property
    def frame_id(self) -> str:
        return self._frame_id
    
    @property
    def init_time(self) -> float:
        return self._init_time
    
    
    def distance_to(self, other) -> float:
        """Calculate the distance between two points in 3d space

        Args:
            other (Pose3D): the other point to calculate distance to

        Raises:
            ValueError: If the other point is not a Pose3D

        Returns:
            float: the distance between the two points in 3d space
        """
        if not isinstance(other, Pose3D):
            raise ValueError("Other must be an instance of Pose3D")
        
        return np.linalg.norm(self.pos - other.pos)
    
    def near_quaternion(self, other, rtol: float = 1e-3, atol: float = 1e-4) -> bool:
        """Check if the quaternion is close to another"""
        
        if not isinstance(other, Pose3D):
            raise ValueError("Other must be an instance of Pose3D")
        
        return tf3d.quaternions.nearly_equivalent(self._rot, other._rot, rtol=rtol, atol=atol)
    
    def is_near(self, other, threshold: float = 0.05, ignore_quaternion: bool=True, rtol: float = 1e-3, atol: float = 1e-4) -> bool:
        """Check if the pose is close to another.

        Args:
            other (Pose3D): the other pose to check if close to.
            threshold (float, optional): the distance threshold for the pose. Defaults to 0.05.
            ignore_quaternion (bool, optional): Whether or not to ignore the quaternion when comparing the poses. Defaults to True.
            rtol (float, optional): rtol value passed to np.allclose() for the quaternion comparison. ignored if ignore_quaternion is True. Defaults to 1e-3.
            atol (float, optional): atol value passed to np.allclose() for the quaternion comparison. ignored if ignore_quaternion is True. Defaults to 1e-4.

        Returns:
            bool: True if the pose is close to the other, otherwise False.
        """
        
        distance = self.distance_to(other)
        if distance < threshold and (not ignore_quaternion or self.near_quaternion(other, rtol=rtol, atol=atol)):
            return True
        
        return False


class Obj3D(object):
    def __init__(self, name: str, max_length: int = 100) -> None:
        """

        Args:
            name (str): Name of the object.
            max_length (int, optional): max amount of stored pose data. Defaults to 100.
            pose_check_threshold (float, optional): Tolerance of the pose distance calculation in meters while checking for target pose. Defaults to 0.1.
        """
        self._name = name
        self._pose_targets = deque()
        self._pose_q = deque(maxlen=max_length)

    @property
    def name(self) -> str:
        return self._name

    @property
    def max_length(self) -> int:
        return self._pose_q.maxlen
    
    @property
    def pose_check_threshold(self) -> float:
        return self._pose_check_threshold
    
    @pose_check_threshold.setter
    def pose_check_threshold(self, value: float):
        self._pose_check_threshold = value
    
    @property
    def current_pose(self) -> Pose3D:
        """ Returns the current pose of the object.
        Assumes that the last added pose is the current one.

        Returns:
            Pose3D: current pose of the object.
        """
        self._pose_q[-1]
    
    @property
    def current_pose_target(self) -> Pose3D:
        """
        Current pose target. This is the pose that will be used for the next pose check.
        """
        return self._pose_targets[0]
        
    def get_all_pose_q(self):
        return list(self._pose_q)
    
    def add_pose(self, pos: List[float], rot: List[float], init_time: float, frame_id: str='relative_frame') -> None:
        """adds a new point to the object.
        if maximum length is reached, the oldest point will be removed.
        """
        self._pose_q.append(Pose3D(pos=pos, rot=rot, init_time=init_time, frame_id=frame_id))
    
    def add_pose_target(self, pos: List[float], rot: List[float], init_time: float, frame_id: str='relative_frame') -> None:
        """ Adds a new position target to the target position queue.

        Args:
            pos (List[float], optional): Position of point in 3d space. Defaults to None.
            rot (List[float], optional): quaterion rotation of point in 3d space. Defaults to None.
            init_time (float, optional): time of initialization. Defaults to None.
            frame_id (str, optional): frame id of point. Defaults to "relative_frame".
        
        """
        self._pose_targets.append(Pose3D(pos=pos, rot=rot, init_time=init_time, frame_id=frame_id))
    
    def update_pose_target(self, threshold: float = 0.05, ignore_quaternion: bool=True, rtol: float = 1e-3, atol: float = 1e-4) -> bool:
        """
        Checks if the current pose target is reached and updates the drone pose target if it is.
        
        returns:
            bool: True if pose target is reached, False otherwise.

        """
        if len(self._pose_targets) > 0:
            pose_target, current_pose = self.current_pose_target, self.current_pose
            if pose_target.is_near(current_pose, threshold=threshold, ignore_quaternion=ignore_quaternion, rtol=rtol, atol=atol):
                self._pose_targets.pop(0)
                return True
        return False
    
    
class DroneController(Node):

    def __init__(self):
        super().__init__("drone_controller")
        
        # drone object instance. Used to track the current pose of the drone.
        self.drone_obj = Obj3D("drone")
        self.avocado_obj = Obj3D("avocado")
        
        # Subscriber to the avocados position relative to the drone. Published by tensorrt node.
        # NOTE: Might need to be changed after further testing. But for now it works.
        self.avocado_pose_sub = self.create_subscription(PoseStamped, "/avocado_pose_target", self.avocado_pose_target_callback, 10)
        
        
        # Drone state messages.
        self.drone_state_messages = []
        # drone state subscriber published by mavros at 1 Hz
        self.drone_state_sub = self.create_subscription(DroneState, '/mavros/state', self.drone_state_callback, STATE_QOS )
        # odometry from ekf3 filter
        self.drone_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.drone_odom_callback, SENSOR_QOS)
        
        # for long command (datastream requests)
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command service not available, waiting again...')

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')

        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')

        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        
        # publisher for setpoint messages. this is how we move the drone.
        self.local_target_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # wait for the drone to be ready for flight
        # NOTE: this is not the most elegant way of doing this but it works for now
        self.ready_for_flight = self.setup_for_flight(timeout=60, wait_for_standby=True, tries=3)
        
        # create a timer to call main_loop. This where we actually send commands to the drone.
        self.main_loop_timer = self.create_timer(0.1, self.main_loop)

    
    def avocado_pose_target_callback(self, msg : PoseStamped):
        
        self.avocado_obj.add_pose(pos=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], 
                                 rot=[msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z], 
                                 init_time=msg.header.stamp.nanosec,
                                 frame_id=msg.header.frame_id)

    def drone_odom_callback(self, msg : Odometry):
        """
        Add a new pose to the drone object.
        """
        self.drone_obj.add_pose(pos=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], 
                                 rot=[msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z], 
                                 init_time=msg.header.stamp.nanosec,
                                 frame_id=msg.header.frame_id)
    
    def drone_state_callback(self, msg : DroneState):
        self.drone_state_messages.append(msg)
        self.get_logger().debug(f"drone state. mode: {msg.mode} armed: {msg.armed}")
        
    def request_data_stream(self, msg_id, msg_interval):
        """Request data stream from the vehicle. Otherwise in some cases (e.g. when using ardupilot sitl), the vehicle will not send any data to mavros.
        
        message ids: https://mavlink.io/en/messages/common.html
        
        Args:
            msg_id: MAVLink message ID
            msg_interval: interval in microseconds
        """
        
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        # wait for response
        rclpy.spin_until_future_complete(self, future)

    def wait_for_new_state_message(self, timeout=30):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.
        
        Args:
            timeout: Seconds to wait until timeout.

        """
        start_len = len(self.drone_state_messages)
        for _ in range(timeout):
            rclpy.spin_once(self)
            if len(self.drone_state_messages) > start_len:
                break

    def change_mode(self, new_mode):
        """ Changes mode the drone.

        Args:
            new_mode: target mode.
        """
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def arm_request(self):
        """ Send an arm request to the drone
        """
        
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self, target_alt):
        """ Takeoff to the target altitude

        Args:
            target_alt: target altitude.
        """
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

    def request_needed(self, msg_interval = 100000):
        """ Requests all needed messages from the fcu.
            uses request_data_stream function.
        
        Args:
            msg_interval: message send interval in microseconds. applies to all data streams called by the function.
        """
        
        # local position
        self.request_data_stream(32, msg_interval)
        # global position
        self.request_data_stream(33, msg_interval)
        # gps raw
        self.request_data_stream(24, msg_interval)
        # raw imu
        self.request_data_stream(27, msg_interval)

    def setup_for_flight(self, timeout=30, wait_for_standby=True, tries=3):
        """ Prepares the drone for flight. returns True if process is succesful returns False otherwise.
        
        Args:
            timeout: seconds wait at each step until timeout. default 30.
            wait_for_standby: whether to wait for standby or not. Defaults to 30.
            tries: How many times the function tries to setup.
        """
        
        # request all needed messages
        self.request_needed()
        
        for i in range(tries):
            if wait_for_standby:
                # wait until system status becomes standby
                for _ in range(timeout):
                    self.wait_for_new_state_message()
                    if self.drone_state_messages[-1].system_status == 3:
                        self.get_logger().info('System status: Standby')
                        break
            
            # change mode to GUIDED
            self.change_mode('GUIDED')
            self.get_logger().info('Mode set to GUIDED')
            
            # try to arm the drone.
            for _ in range(timeout):
                # send an arm request
                self.arm_request()
                self.get_logger().info('Arming request sent.')
                
                # wait for a new state message 
                self.wait_for_new_state_message()
                if self.drone_state_messages[-1].armed:
                    self.get_logger().info('Arming successful')
                    break
            
            # check if the drone is ready or not
            self.wait_for_new_state_message(timeout)
            
            last_state = self.drone_state_messages[-1]
            
            if last_state.armed and last_state.guided:
                self.get_logger().info("Drone is ready for flight")
                return True
        
            self.get_logger().info(f"Drone Setup failed armed:{last_state.armed} guided:{last_state.guided} mode:{last_state.mode}")
        
        return False

    def go_to_local(self, position, rot_q):
        
        msg = PoseStamped()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        msg.pose.orientation.w = rot_q[0]
        msg.pose.orientation.x = rot_q[1]
        msg.pose.orientation.y = rot_q[2]
        msg.pose.orientation.z = rot_q[3]
        
        self.local_target_pub.publish(msg)
        self.get_logger().info(f"moving to position:{position[0], position[1], position[2]} , orientation:{rot_q[0], rot_q[1], rot_q[2], rot_q[3]}")


    def main_loop(self):
        
        # check if the drone is ready or not
        if not self.ready_for_flight():
            self.get_logger().debug("Waiting for drone to be ready...")
            return
        
       

def main():
    rclpy.init()
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
