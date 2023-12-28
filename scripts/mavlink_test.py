from pymavlink import mavutil
import time
import math
import threading


class Drone(object):
    def __init__(self, master):
        self.position_history_global = []
        self.position_history_local = []
        self.master = master

        self.counter = 0 
        self.start_time = time.time()
        

    def sync(self):
        self.counter = time.time() - self.start_time
        return self.counter
        
    def update_position_global(self, time, lat, lon, alt, relative_alt, x_velocity, y_velocity, z_velocity, yaw_angle):
        position = {
            'time': time,
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'relative_alt': relative_alt,
            'x_velocity': x_velocity,
            'y_velocity': y_velocity,
            'z_velocity': z_velocity,
            'yaw_angle': yaw_angle
        }
        self.position_history_global.append(position)

    def update_position_local(self, time, x, y, z, vx, vy, vz):
        local_pos = {
            'time':time,
            'x': x,
            'y': y,
            'z': z,
            'vx' : vx,
            'vy' : vy,
            'vz' : vz
        }    

        self.position_history_local.append(local_pos)

    #returns the last known position of the drone.
    def get_pos_global(self,idx, return_as_list = False, no_time = False, no_pos = False, no_velocity = False, no_yaw = False):

        if len(self.position_history_global) == 0:
           raise IndexError("No Position Data Was Found")
        elif  idx >= len(self.position_history_global):
            raise IndexError("Index Out Of Range!")
     
        last_pos = self.position_history_global[idx]
        result = []
        if not no_time:
            result.append(last_pos['time'])
        if not no_pos:
            result.append(last_pos['lat'])    
            result.append(last_pos['lon']) 
            result.append(last_pos['alt']) 
            result.append(last_pos['relative_alt']) 
        if not no_velocity:
            result.append(last_pos['x_velocity']) 
            result.append(last_pos['y_velocity']) 
            result.append(last_pos['z_velocity']) 
        if not no_yaw:
            result.append(last_pos['yaw_angle'])     
        
        if return_as_list:
            return result
        return *result,

    def get_pos_local(self, idx, no_velocity = True):
        if len(self.position_history_local) == 0:
           raise IndexError("No Position Data Was Found")
        elif  idx >= len(self.position_history_local):
            raise IndexError("Index Out Of Range!")

        pos = self.position_history_local[idx]
        result = []

        result.append(pos['x'])
        result.append(pos['y'])
        result.append(pos['z'])

        if not no_velocity:
            result.append(pos['vx'])
            result.append(pos['vy'])
            result.append(pos['vz'])

        return *result,     


    def set_mode(self):
        master = self.master
        m = master.mav.command_long_encode(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,
            4,
            0, 0, 0, 0, 0 #empty
        )
        master.mav.send(m)

    # instruct the drone to send messages needed.
    def request(self):
        master = self.master
        #global_pos
        global_position_message = master.mav.command_long_encode(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        33, # msg
        100000, #interval microseconds
        0,0,0,0,0,   #unused
        )
        master.mav.send(global_position_message)

        #local
        global_position_message = master.mav.command_long_encode(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        32, # msg LOCAL_POSITION_NED 
        100000, #interval microseconds
        0,0,0,0,0,   #unused
        )
        master.mav.send(global_position_message)




    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return newlat, newlon, original_location.alt

    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def goto_target_local(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to move the drone to specified location
        North, East, Down

        Down values are negative. -10 means 10 meters above, 10 means 10 meters below
        """
        master = self.master
        msg = master.mav.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        master.target_system, master.target_component,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        master.mav.send(msg)

    def arm(self):
        master = self.master    
        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, #confirm
        1, # arm par1
        0, 0, 0, 0, 0, 0, 
        )

        
    def take_off(self, altitude = 5):
        master = self.master
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, #confirmation
            0, 0, 0, 0 , 0, 0, # empty
            altitude
        )    

        #move the vehicle to the starting position then land
    def move_to_landing_pos(self, offset = 0.2, descend_rate = 2):
        master = self.master

        #get the first stored local_position
        x, y, z = self.get_pos_local(idx = 0, no_velocity = True)
        
        # TODO: just lands at the current location.
        # fix it so that drone moves on top of the landing pos first.
        # maybe make them separate
        # land to the current pos.
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    
    

"""
TODO: add functions to align the drone before landing directly. 
Zed camera script is already ready just need to merge them.

"""


def main():
    error_messages = {
        0: "Command executed successfully.",
        1: "Command cannot be executed at this time. Retry later.",
        2: "Invalid parameters. Retry with different parameters.",
        3: "Command is not supported.",
        4: "Execution of command failed. Fix the issue before retrying.",
        5: "Command is in progress. Further updates expected.",
        6: "Command has been cancelled.",
        7: "Command is only accepted as COMMAND_LONG.",
        8: "Command is only accepted as COMMAND_INT.",
        9: "Invalid command frame specified.",
    }

    command_codes = {
        176:'MAV_CMD_DO_SET_MODE',
        511:'MAV_CMD_SET_MESSAGE_INTERVAL',
        32: 'LOCAL_POSITION_NED',
        85: 'POSITION_TARGET_LOCAL_NED',
        22: 'MAV_CMD_NAV_TAKEOFF',
        21: 'MAV_CMD_NAV_LAND',
        400: 'MAV_CMD_COMPONENT_ARM_DISARM',
    }

    # Connect to the drone
    master = mavutil.mavlink_connection("tcp:127.0.0.1:5762") 
    master.wait_heartbeat()
    print(f"Connected to system:{master.target_system} component:{master.target_component}")

    #  Create the drone class instance
    drone = Drone(master=master)

    """
    flight plan, tells the code to when to execute what command. 
    When a command is executed that key gets removed.

    keys: function to execute
    value: amaount of time that has to pass to execute the command.
    
    NOTE: can use the function name instead of directly storing the funcs.
          then use them later with eval()
    """
    flight_plan = {
        drone.set_mode: ((), {}, 0.5), # function: ((), {'kwarg1': 0.5, 'kwarg2': 2}, execution_time)
        drone.request: ((), {}, 1),
        drone.arm: ((), {}, 2),
        drone.take_off: ((), {'altitude': 10}, 5),  
        drone.goto_target_local: ((), {'north':5, 'east':0, 'down':0}, 12),
        drone.move_to_landing_pos: ((), {'offset': 0.5, 'descend_rate': 2}, 25),  
    }

    current_time = drone.sync()

    while True:
        # Listen for message
        msg = master.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # update global pos
                drone.update_position_global( time=msg.time_boot_ms, lat=msg.lat, lon=msg.lon, alt=msg.alt, relative_alt=msg.relative_alt,
                   x_velocity=msg.vx, y_velocity=msg.vy, z_velocity=msg.vz, yaw_angle=msg.hdg)       

            if msg.get_type() == 'LOCAL_POSITION_NED':
                #update local pos
                drone.update_position_local(time=msg.time_boot_ms, x=msg.x,y=msg.y, z=msg.z, vx=msg.vx,vy=msg.vy, vz=msg.vz)
                           
            #handle response messages
            if msg.get_type() == 'COMMAND_ACK':
                result_code = msg.result
                response_code = msg.command

                if result_code in error_messages and response_code in command_codes:
                    print(error_messages[result_code] + f" | {command_codes[response_code]}")
                else:
                    print(f"Unknown Code/codes: result_code:{result_code} command_code:{response_code}")

        #------------------------------------------------------------------------------------------------#

        current_time = drone.sync() # time passed since start
        commands_to_remove = []

        for func, (args, kwargs, execution_time) in flight_plan.items():
            if (current_time - 0.05) >= execution_time:  # Check if it's time for the command to execute
                print(f"Executing command -> time: {current_time:.2f} -> {func.__name__} ")
                if kwargs:  # If there are keyword arguments
                    func(*args, **kwargs)  # Call the function with arguments and keyword arguments
                else:
                    func(*args)  # Call the function with arguments
                commands_to_remove.append(func)

        # Remove executed commands from flight_plan after the loop
        for func in commands_to_remove:
            del flight_plan[func]


        if current_time > 35:
            break
    
    #print pos values after runtime
    for e in drone.position_history_local:
        print(f"x:{e['x']:.2f} y:{e['y']:.2f} z:{e['z']:.2f}")


if __name__ == "__main__":
    main()


