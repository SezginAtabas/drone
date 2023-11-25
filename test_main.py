import cv2
import random
import time
import numpy as np

import tensorrt as trt    
import pycuda.driver as cuda
import pycuda.autoinit

import pyzed.sl as sl
from pymavlink import mavutil


from trt_detect import yolov8_trt, plot_one_box



class LandingManager(object):
    def __init__(self, master):
        self.master = master

    def check_alignment(self):
        master = self.master

        #request position
        master.mav.request_position_target_global_int_send(
        master.target_system, master.target_component, 0, 0)

        # Receive the feedback
        msg = master.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True, timeout=1)
        if msg is not None:
            print(msg)
        else:
            print("didint receive position")




class ZedDetection(object):
    def __init__(self):
        self.current_xyz = []


    def detect(self):

        engine_file = 'best130_192_320.engine'
        INPUT_SIZE = (192,320)
        OUTPUT_SHAPE = [1,5,1260]
        
        #open zed camera and set the parameters
        zed = sl.Camera()
        init_params = sl.InitParameters()  
        init_params_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.sdk_verbose = 1
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.depth_maximum_distance = 10
        
        #check if camera opened without errors
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        #create sl.Mat objects
        img_zed= sl.Mat()
        depth = sl.Mat()
        #set zed camera settings
        zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 70)
        zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 6)

        y_trt = yolov8_trt(engine_file_path=engine_file,input_size=INPUT_SIZE, output_shape=OUTPUT_SHAPE)
        
        found_img = False
        
        while zed.grab() == sl.ERROR_CODE.SUCCESS:
            #grab an image in sl.mat
            zed.retrieve_image(img_zed, sl.VIEW.LEFT)
            #use get_data() to get the numpy array
            img_np = img_zed.get_data()
            frame = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
        
            #run inference
            res,box,conf,found_img = y_trt.infer(input_img=frame)
            
            #if an img is found
            if found_img:     
                
                #center of the detect objects bounding box
                center_point_x = int((box[0] + box[2]) / 2)
                center_point_y = int((box[1] + box[3]) / 2)

                #find the depth at the center of the target
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                err,depth_value = depth.get_value(center_point_x,center_point_y)

                #get the camera information
                #Might not need to run during the runtime constantly 
                #never saw the values chnage. but beter safe than sorry.
                camera_info = zed.get_camera_information()
                calibration_params = camera_info.camera_configuration.calibration_parameters


                #camera calibration parameters to find the coordinates
                f_x = calibration_params.left_cam.fx
                f_y = calibration_params.left_cam.fy
                c_x = calibration_params.left_cam.cx
                c_y = calibration_params.left_cam.cy
                
                #point that the depth is taken
                u, v = center_point_x, center_point_y

                #convert to 3d coordinates
                Z = depth_value
                X = ((u - c_x) * Z) / (f_x)
                Y = ((v - c_y) * Z) / (f_y)

                self.current_xyz = [X, Y, Z]

                #label_text = f"x:{X:.2f} y:{Y:.2f} z:{Z:.2f}"
                #plot the results
                #plot_one_box(box,res,label=label_text)
                    
        
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            
            #cv2.imshow("vid",res)

        
        #close everything at the end
        zed.close()  
        cv2.destroyAllWindows()  


def main():
    #connection
    master = mavutil.mavlink_connection("/dev/ttyACM0",baud=57600)
    #wait for a heartbeat the confirm the connection
    master.wait_heartbeat()
    print(f"Connected. system:{master.target_system} component:{master.target_component}")
    
    #create the LandingManager instance
    Lander = LandingManager(master=master)

    #create a ZedDetection instance
    detection = ZedDetection()

    

if __name__ == "__main__":
    main()