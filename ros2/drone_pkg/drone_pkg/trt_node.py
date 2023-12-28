
import rclpy 
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

import cv2
import numpy as np
import time
import random
from cv_bridge import CvBridge

import tensorrt as trt    
import pycuda.driver as cuda
import pycuda.autoinit


# ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera model>




# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.qos.qos_profile_sensor_data



#gets the data needed for the TensorRT model to run from the ZED camera
class MyGetZedInfo(Node):
    def __init__(self):
        super().__init__('my_get_zed_info')

        self.depth_map = []
        self.rgb_img = []
        self.last_published_lading_pos = PointStamped()
        self.last_published_avo_pos = PointStamped()
        self.current_detection_target = "landing_pad" # which target to detect. can be avocado or landing_pad

        self.target_landing_pos = {'x' : 0, 'y': 0, 'z': 0 }
        self.last_avo_pos = {'x' : 0, 'y': 0, 'z': 0 }
        
        self.bridge = CvBridge()
        
        # tensorrt engine file paths
        engine_file = '/home/xtrana/ros2_ws/src/drone_pkg/trt_engines/landing_134.engine'

        # engine file for the avocado
        avo_engine_file = '/home/xtrana/ros2_ws/src/drone_pkg/trt_engines/avo_n_42.engine'

        INPUT_SIZE = (384, 640)
        OUTPUT_SHAPE = [1, 5, 5040]

        # create the tensorrt class instance
        self.landing_trt = yolov8_trt(engine_file_path=engine_file, input_size=INPUT_SIZE, output_shape=OUTPUT_SHAPE)
        # create another instance for the avocado
        self.avo_trt = yolov8_trt(engine_file_path=avo_engine_file, input_size=INPUT_SIZE, output_shape=OUTPUT_SHAPE)

        self.rgb_img_width = 0
        self.rgb_img_height = 0
        self.depth_map_width = 0
        
        self.f_x = 0
        self.f_y = 0
        self.c_x = 0
        self.c_y = 0

        

        
        #QoS profile for depth topic
        depth_qos = rclpy.qos.QoSProfile(depth=10)
        depth_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        depth_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        #QoS profile for rgb rectified image
        rgb_img_qos = rclpy.qos.QoSProfile(depth=10)
        rgb_img_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        rgb_img_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        camera_info_qos = rclpy.qos.QoSProfile(depth=10)
        camera_info_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        camera_info_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        

        # sub for current detection target
        self.current_target_sub = self.create_subscription(
            String, 
            '/my_drone/current_detection_target',
            self.current_detection_target_callback,
            10
        )
  
        #create the depth map subscriber
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            depth_qos
        )

        self.rgb_img_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.rgb_img_callback,
            rgb_img_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            camera_info_qos
        )


        # create a publisher for the target landing position
        self.target_landing_pos_pub = self.create_publisher(PointStamped, '/my_drone/landing_target_position', qos_profile=SENSOR_QOS)
        # publisher for avocado position
        self.avocado_pos_pub = self.create_publisher(PointStamped, '/my_drone/avocado_target_position', qos_profile=SENSOR_QOS)



    def current_detection_target_callback(self, msg):
        self.current_detection_target = msg.data
        self.get_logger().info(f"current detection target: {self.current_detection_target}")
    
    def camera_info_callback(self, msg):
        self.f_x = msg.k[0]
        self.f_y = msg.k[4]
        self.c_x = msg.k[2]
        self.c_y = msg.k[5]

    def depth_callback(self, msg):
        # Get a pointer to the depth values casting the data pointer to floating point
        self.depth_map_width = msg.width
        self.depth_map = memoryview(msg.data).cast('f')


    def rgb_img_callback(self, msg):
        # convert the Image msg to cv2 image

        self.rgb_img_width = msg.width
        self.rgb_img_height = msg.height

        self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='bgr8')

        # run the detection for the image
        self.detect()


    # gets called when rbg image is received. currently only runs detection for the landing target.
    def detect(self):
            
            input_img  = self.rgb_img
            input_depth_map = self.depth_map


            im_width =  self.rgb_img_width
            im_height = self.rgb_img_height
            depth_width = self.depth_map_width

            f_x = self.f_x
            f_y = self.f_y
            c_x = self.c_x
            c_y = self.c_y
            

            # store the current_detection_target in a variable to aviod changing it while running the detection
            current_detection_target = self.current_detection_target


            

            # detection for the landing pad
            if current_detection_target == "landing_pad":
                self.get_logger().info("running trt detection for landing pad ... ")
                # inference
                raw_img, box, conf, target_found = self.landing_trt.infer(input_img=input_img) 

                if target_found: 
                    # find the center point of the bbox
                    u = int((box[0] + box[2]) / 2) # x
                    v = int((box[1] + box[3]) / 2) # y

                    # linear index of the center pixel of bbox
                    center_idx = u + depth_width * v
                    

                    # real world distance from the target
                    Z = input_depth_map[center_idx]
                    X = ((u - c_x) * Z) / (f_x)
                    Y = ((v - c_y) * Z) / (f_y)

                    self.target_landing_pos['x'] = X
                    self.target_landing_pos['y'] = Y
                    self.target_landing_pos['z'] = Z

                    # publish the target landing position
                    self.last_published_lading_pos.header.stamp = self.get_clock().now().to_msg()
                    self.last_published_lading_pos.header.frame_id = "d_landing_pos"
                    self.last_published_lading_pos.point.x = X
                    self.last_published_lading_pos.point.y = Y
                    self.last_published_lading_pos.point.z = Z
                    
                    # publish the target landing position
                    self.target_landing_pos_pub.publish(self.last_published_lading_pos)

                    self.get_logger().info(f"x:{X:.2f} y:{Y:.2f} z:{Z:.2f}")

                

            # detection for the avocado
            elif self.current_detection_target == "avocado":
                self.get_logger().info("running trt detection for avocado ... ")
                # inference
                raw_img, box, conf, target_found = self.avo_trt.infer(input_img=input_img)

                if target_found:
                    # find the center point of the bbox
                    u = int((box[0] + box[2]) / 2)
                    v = int((box[1] + box[3]) / 2)

                    # linear index of the center pixel of bbox
                    center_idx = u + depth_width * v

                    # real world distance from the target
                    Z = input_depth_map[center_idx]
                    X = ((u - c_x) * Z) / (f_x)
                    Y = ((v - c_y) * Z) / (f_y)

                    self.last_avo_pos['x'] = X
                    self.last_avo_pos['y'] = Y
                    self.last_avo_pos['z'] = Z

                    # publish the avocado position
                    self.last_published_avo_pos.header.stamp = self.get_clock().now().to_msg()
                    self.last_published_avo_pos.header.frame_id = "d_avo_pos"
                    self.last_published_avo_pos.point.x = X
                    self.last_published_avo_pos.point.y = Y
                    self.last_published_avo_pos.point.z = Z

                    # publish the avocado position
                    self.avocado_pos_pub.publish(self.last_published_avo_pos)

                    self.get_logger().info(f"x:{X:.2f} y:{Y:.2f} z:{Z:.2f}")

        


class yolov8_trt(object):
    #initiate the engine and setup stuff
    def __init__(self, engine_file_path, input_size, output_shape):
        self.ctx = cuda.Device(0).make_context()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)
        stream = cuda.Stream()        

        #deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())  

        self.context = engine.create_execution_context() 


        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []


        #allocate memory for input and output
        for binding in engine:
            #create page-locked memory buffers and allocate memory on host and device
            size = trt.volume(engine.get_tensor_shape(binding))
            host_mem = cuda.pagelocked_empty(size, np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
        
             #append the device buffer to device bindings
            bindings.append(int(cuda_mem))
            #append the binding the the input or output list
            if binding == "images":
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)




        # store the values
        self.stream = stream
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self.input_size = input_size
        self.output_shape = output_shape

    
    def infer(self, input_img):

        # push the context to the gpu
        self.ctx.push()

        #set the values
        input_size = self.input_size
        output_shape = self.output_shape
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        stream = self.stream
        context = self.context
        
        input_img_raw = input_img
        
       
                
                
        #preprocess
        input_img, _, _, _ = self.preprocess_image(input_img_raw,input_size[0],input_size[1])
        #copy input image to host buffer
        np.copyto(host_inputs[0],input_img.ravel())    
        #start the counter  
        start = time.time()
        #transfer the input data to gpu for execution
        cuda.memcpy_htod_async(cuda_inputs[0],host_inputs[0],stream)    
         #run inference
        context.execute_async_v2(bindings=bindings,stream_handle=stream.handle)
        #transfer predicitions from the gpu
        cuda.memcpy_dtoh_async(host_outputs[0],cuda_outputs[0],stream)
        stream.synchronize()

        #end the counter
        end = time.time()

        # remove the context from the gpu
        self.ctx.pop()




        #amount of time spent
        time_spent = str(end - start)
        output = host_outputs[0]     
        

        final_output = input_img_raw
        
        output = output.reshape(output_shape[0],output_shape[1],output_shape[2])
        results = self.postprocess(preds = output,img = input_img,orig_img =  input_img_raw,
        OBJ_THRESH = 0.5,NMS_THRESH = 0.3)   
        
        
        box = []
        conf = 0.0
        
        try:
            results = results[0][0][0]  
        except:
            #didnt find img return false
            return final_output, box, conf, False    
        box = results[:4]
        conf = results[4]
        cls = results[5] 
        #plot_one_box(box,final_output,label="helipad")      
        
        #found an iamge return true for found_img
        return final_output,box,conf,True       

    def destroy(self):
        #remove the context from the gpu
        self.ctx.pop()

    def preprocess_image(self, raw_bgr_image,input_h,input_w):
            """
            description: Convert BGR image to RGB,
                        resize and pad it to target size, normalize to [0,1],
                        transform to NCHW format.
            param:
                input_image_path: str, image path
            return:
                image:  the processed image
                image_raw: the original image
                h: original height
                w: original width
            """
            image_raw = raw_bgr_image
            h, w, c = image_raw.shape
            image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
            # Calculate widht and height and paddings
            r_w = input_w / w
            r_h = input_h / h
            if r_h > r_w:
                tw = input_w
                th = int(r_w * h)
                tx1 = tx2 = 0
                ty1 = int((input_h - th) / 2)
                ty2 = input_h - th - ty1
            else:
                tw = int(r_h * w)
                th = input_h
                tx1 = int((input_w - tw) / 2)
                tx2 = input_w - tw - tx1
                ty1 = ty2 = 0
            # Resize the image with long side while maintaining ratio
            image = cv2.resize(image, (tw, th))
            # Pad the short side with (128,128,128)
            image = cv2.copyMakeBorder(
                image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, None, (128, 128, 128)
            )
            image = image.astype(np.float32)
            # Normalize to [0,1]
            image /= 255.0
            # HWC to CHW format:
            image = np.transpose(image, [2, 0, 1])
            # CHW to NCHW format
            image = np.expand_dims(image, axis=0)
            # Convert the image to row-major order, also known as "C order":
            image = np.ascontiguousarray(image)
            return image, image_raw, h, w

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
            
        """
        description: Plots one bounding box on image img,
                    this function comes from YoLov8 project.
        param: 
            x:      a box likes [x1,y1,x2,y2]
            img:    a opencv image object
            color:  color to draw rectangle, such as (0,255,0)
            label:  str
            line_thickness: int
        return:
            no return

        """
        tl = (
                line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        )  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(
                img,
                label,
                (c1[0], c1[1] - 2),
                0,
                tl / 3,
                [225, 255, 255],
                thickness=tf,
                lineType=cv2.LINE_AA,
            )
    def xywh2xyxy(self, x):
        y = np.copy(x)
        y[..., 0] = x[..., 0] - x[..., 2] / 2  # top left x
        y[..., 1] = x[..., 1] - x[..., 3] / 2  # top left y
        y[..., 2] = x[..., 0] + x[..., 2] / 2  # bottom right x
        y[..., 3] = x[..., 1] + x[..., 3] / 2  # bottom right y
        return y

    def clip_boxes(self, boxes, shape):
        boxes[..., [0, 2]] = boxes[..., [0, 2]].clip(0, shape[1])  # x1, x2
        boxes[..., [1, 3]] = boxes[..., [1, 3]].clip(0, shape[0])  # y1, y2

    def scale_boxes(self, img1_shape, boxes, img0_shape, ratio_pad=None):
        if ratio_pad is None:  # calculate from img0_shape
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]

        boxes[..., [0, 2]] -= pad[0]  # x padding
        boxes[..., [1, 3]] -= pad[1]  # y padding
        boxes[..., :4] /= gain
        self.clip_boxes(boxes, img0_shape)
        return boxes

    def crop_mask(self, masks, boxes):
        n, h, w = masks.shape
        x1, y1, x2, y2 = np.split(boxes[:, :, None], 4, axis=1)
        r = np.arange(w, dtype=np.float32)[None, None, :]  # rows shape(1,w,1)
        c = np.arange(h, dtype=np.float32)[None, :, None]  # cols shape(h,1,1)

        return masks * ((r >= x1) * (r < x2) * (c >= y1) * (c < y2))

    def sigmoid(self, x): 
        return 1.0/(1+np.exp(-x))

    def nms(self, bboxes, scores, threshold=0.5):
        x1 = bboxes[:, 0]
        y1 = bboxes[:, 1]
        x2 = bboxes[:, 2]
        y2 = bboxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)

        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            if order.size == 1: break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, (xx2 - xx1))
            h = np.maximum(0.0, (yy2 - yy1))
            inter = w * h

            iou = inter / (areas[i] + areas[order[1:]] - inter)
            ids = np.where(iou <= threshold)[0]
            order = order[ids + 1]

        return keep

    def non_max_suppression(
            self,
            prediction,
            conf_thres=0.25,
            iou_thres=0.45,
            classes=None,
            agnostic=False,
            multi_label=False,
            labels=(),
            max_det=300,
            nc=0,  # number of classes (optional)
    ):

        # Checks
        assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
        assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'
        #print(prediction.shape)
        #【lulu】prediction.shape[1]：box + cls + num_masks
        bs = prediction.shape[0]              # batch size
        nc = nc or (prediction.shape[1] - 4)  # number of classes
        nm = prediction.shape[1] - nc - 4     # num_masks
        mi = 4 + nc                           # mask start index
        xc = np.max(prediction[:, 4:mi], axis=1) > conf_thres ## 【lulu】

        # Settings
        # min_wh = 2  # (pixels) minimum box width and height
        max_wh = 7680  # (pixels) maximum box width and height
        max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
        time_limit = 0.5 + 0.05 * bs  # seconds to quit after
        redundant = True  # require redundant detections
        multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
        merge = False  # use merge-NMS

        t = time.time()
        output = [np.zeros((0,6 + nm))] * bs ## 【lulu】

        for xi, x in enumerate(prediction):  # image_3c index, image_3c inference
            # Apply constraints
            # x[((x[:, 2:4] < min_wh) | (x[:, 2:4] > max_wh)).any(1), 4] = 0  # width-height

            x = np.transpose(x,[1, 0])
            x = x[xc[xi]] ## 【lulu】

            # If none remain process next image_3c
            if not x.shape[0]: continue


            # Detections matrix nx6 (xyxy, conf, cls)
            box, cls, mask = np.split(x, [4, 4+nc], axis=1) ## 【lulu】
            box = self.xywh2xyxy(box)  # center_x, center_y, width, height) to (x1, y1, x2, y2)

            j = np.argmax(cls, axis=1)  ## 【lulu】
            conf = cls[np.array(range(j.shape[0])), j].reshape(-1,1)
            x = np.concatenate([box, conf, j.reshape(-1,1), mask], axis=1)[conf.reshape(-1,)>conf_thres]

            # Check shape
            n = x.shape[0]  # number of boxes
            if not n: continue
            x = x[np.argsort(x[:, 4])[::-1][:max_nms]]  # sort by confidence and remove excess boxes 【lulu】

            # Batched NMS
            c = x[:, 5:6] * max_wh  # classes
            boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
            i = self.nms(boxes, scores, iou_thres) ## 【lulu】
            i = i[:max_det]  # limit detections

            output[xi] = x[i]
            if (time.time() - t) > time_limit:
                # LOGGER.warning(f'WARNING ⚠️ NMS time limit {time_limit:.3f}s exceeded')
                break  # time limit exceeded
        return output

    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, ratio, (dw, dh)        

    def postprocess(self, preds, img, orig_img, OBJ_THRESH, NMS_THRESH, classes=None):
        p = self.non_max_suppression(preds,
                                    OBJ_THRESH,
                                    NMS_THRESH,
                                    agnostic=False,
                                    max_det=300,
                                    nc=classes,
                                    classes=None)        
        results = []
        for i, pred in enumerate(p):
            shape = orig_img.shape
            if not len(pred):
                results.append([[], []])  # save empty boxes
                continue
            pred[:, :4] = self.scale_boxes(img.shape[2:], pred[:, :4], shape).round()
            results.append([pred[:, :6], shape[:2]])
        return results

    def gen_color(self, class_num):
        color_list = []
        np.random.seed(1)
        while 1:
            a = list(map(int, np.random.choice(range(255),3)))
            if(np.sum(a)==0): continue
            color_list.append(a)
            if len(color_list)==class_num: break
        return color_list


def main(args=None):
    rclpy.init(args=args)

    info_node = MyGetZedInfo()


    rclpy.spin(info_node)


    cv2.destroyAllWindows()    
    info_node.destroy_node()
    rclpy.shutdown()        

if __name__ == "__main__":
    main()