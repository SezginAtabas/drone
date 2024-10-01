import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

import cv2
import numpy as np
import time
import random
from cv_bridge import CvBridge

import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# import norfair tracker
from norfair import Detection, Tracker
from norfair.tracker import TrackedObject

# ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera model>

# SENSOR_QOS used for most sensor streams
SENSOR_QOS = rclpy.qos.qos_profile_sensor_data


class TensorrtNode(Node):
    def __init__(self):
        super().__init__("tensorrt_node")

        input_size = (384, 640)
        output_shape = [1, 5, 5040]

        # tensorrt engine file paths
        engine_file = ""
        avo_engine_file = ""

        # rgb img variables
        self.rgb_img = []
        self.rgb_img_width = 0
        self.rgb_img_height = 0
        # vas for depth image
        self.depth_map = []
        self.depth_map_width = 0

        # zed camera info. For image correction.
        self.f_x = 0
        self.f_y = 0
        self.c_x = 0
        self.c_y = 0

        # last published position for landing and avo. can be used to help the drone find the targets if drone loses them.
        # TODO: Instead of just saving last know position. Save all detections and drones position at the time of the detection
        self.last_published_lading_pos = PointStamped()
        self.last_published_avo_pos = PointStamped()
        self.current_detection_target = (
            "landing_pad"  # which target to detect. can be avocado or landing_pad
        )

        self.target_landing_pos = {"x": 0, "y": 0, "z": 0}
        self.last_avo_pos = {"x": 0, "y": 0, "z": 0}

        # variables to check if all need messages are received for the detection to start
        self.received_depth_image = False
        self.received_rgb_image = False
        self.received_camera_info = False

        self.drone_local_position = PoseStamped()

        # bridge for converting ros img messages to opencv images.
        self.bridge = CvBridge()

        # create the norfair tracker
        try:
            # NOTE: These parameters are for testing and are not final.
            self.landing_tracker = Tracker(
                distance_function="euclidean",
                distance_threshold=1000.0,
                hit_counter_max=300,
            )
            self.avo_tracker = Tracker(
                distance_function="euclidean",
                distance_threshold=1000.0,
                hit_counter_max=300,
            )

        except Exception as e:
            self.get_logger().error(f"Error Loading tracker! - {e}")
            exit()

        try:
            # create the tensorrt class instance
            self.landing_trt = yolov8_trt(
                engine_file_path=engine_file,
                input_size=input_size,
                output_shape=output_shape,
            )
            # create another instance for the avocado
            self.avo_trt = yolov8_trt(
                engine_file_path=avo_engine_file,
                input_size=input_size,
                output_shape=output_shape,
            )

        except FileNotFoundError as e:
            self.get_logger().error(
                f"Couldn't find the TensorRT engines. error - {e} \n Exiting ..."
            )

        except Exception as e:
            self.get_logger().error(
                f"Couldn't create tensorrt engines. error - {e} \n Exiting ..."
            )
            exit()

        # QoS profile for depth topic
        depth_qos = rclpy.qos.QoSProfile(depth=10)
        depth_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        depth_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        # QoS profile for rgb rectified image
        rgb_img_qos = rclpy.qos.QoSProfile(depth=10)
        rgb_img_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        rgb_img_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        # QoS profile for zed camera info
        camera_info_qos = rclpy.qos.QoSProfile(depth=10)
        camera_info_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        camera_info_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        # sub for current detection target
        self.current_target_sub = self.create_subscription(
            String,
            "/my_drone/current_detection_target",
            self.current_detection_target_callback,
            10,
        )
        # create the depth map subscriber
        self.depth_sub = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.depth_callback,
            depth_qos,
        )

        self.rgb_img_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",
            self.rgb_img_callback,
            rgb_img_qos,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "zed/zed_node/rgb/camera_info",
            self.camera_info_callback,
            camera_info_qos,
        )
        # sub for the drones local position used to sync detections.
        self.pos_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_position_callback,
            SENSOR_QOS,
        )

        # create a publisher for the target landing position
        self.target_landing_pos_pub = self.create_publisher(
            PointStamped, "/my_drone/landing_target_position", qos_profile=SENSOR_QOS
        )
        # publisher for avocado position
        self.avocado_pos_pub = self.create_publisher(
            PointStamped, "/my_drone/avocado_target_position", qos_profile=SENSOR_QOS
        )

        # call detection at 30
        self.detect_timer = self.create_timer(
            timer_period_sec=1 / 30, callback=self.detect
        )

    def local_position_callback(self, msg):
        self.drone_local_position = msg

    def current_detection_target_callback(self, msg):
        self.current_detection_target = msg.data
        self.get_logger().info(
            f"current detection target: {self.current_detection_target}"
        )

    def camera_info_callback(self, msg):

        if not self.received_camera_info:
            self.get_logger().info("received camera information.")
            self.received_camera_info = True

        self.f_x = msg.k[0]
        self.f_y = msg.k[4]
        self.c_x = msg.k[2]
        self.c_y = msg.k[5]

    def depth_callback(self, msg):
        # check if this depth message the first one.
        if not self.received_depth_image:
            self.get_logger().info(f"First Depth Image Received")
            self.received_depth_image = True

        self.depth_map_width = msg.width
        # Get a pointer to the depth values casting the data pointer to floating point
        self.depth_map = memoryview(msg.data).cast("f")

    def rgb_img_callback(self, msg):
        # convert the Image msg to cv2 image

        if not self.received_rgb_image:
            self.get_logger().info(f"first rgb image received")
            self.received_rgb_image = True

        self.rgb_img_width = msg.width
        self.rgb_img_height = msg.height

        self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        # run the detection for the image
        # self.detect()

    def yolo_to_norfair(self, yolo_detections):
        """
        converts yolo detections to nofair Detection to be fed into norfair tracking.
        """

        norfair_detections: list[Detection] = []
        for yolo_d in yolo_detections:
            bbox = np.array(
                [
                    [yolo_d[0].item(), yolo_d[1].item()],
                    [yolo_d[2].item(), yolo_d[3].item()],
                ]
            )
            scores = np.array([yolo_d[4].item(), yolo_d[4].item()])

            norfair_detections.append(
                Detection(points=bbox, scores=scores, label=int(yolo_d[-1].item()))
            )

        return norfair_detections

    def pred_to_dist(
        self, img_shape, depth_map, depth_width, camera_info, radius, predictions
    ):
        """Calculates the 3d positions of the input predictions.

        Args:
            img_shape: shape of the image. tuple (width, height)
            depth_map:  depth_map that will be used for 3d position calculations
            depth_width:  width of the depth map
            camera_info: information need for the calculation. (f_x, f_y, c_x, c_y)
            radius: Radius around pixel that will be used for depth calculation.
            predictions : list of predictions. [[x1, y1, x2, y2 , conf, cls], ... ]

        Returns:
            list containing results [x, y, z, conf, cls]
        """
        f_x = camera_info[0]
        f_y = camera_info[1]
        c_x = camera_info[2]
        c_y = camera_info[3]

        out = []  # list containing results [x, y, z, cls]
        for prediction in predictions:
            box = prediction[:4]
            # center point of the bounding box
            center_u = int((box[0] + box[2]) / 2)  # x
            center_v = int((box[1] + box[3]) / 2)  # y

            # get the pixels within a certain radius around the center and get their average depth
            pixel_cords = self.find_pixels_near_center(
                width=img_shape[0],
                height=img_shape[1],
                center_coords=(center_u, center_v),
                radius=radius,
            )

            total_depth = 0.0
            for cords in pixel_cords:
                # linear index of the pixel
                lin_idx = cords[0] + depth_width * cords[1]
                # add the depth of the pixel to the total
                if not np.isnan(depth_map[lin_idx]):
                    total_depth += depth_map[lin_idx]

            # average depth of the pixels
            z = total_depth / len(pixel_cords)
            x = ((center_u - c_x) * z) / (f_x)
            y = ((center_v - c_y) * z) / (f_y)
            out.append([x, y, z, prediction[4], prediction[5]])

        return out

    def find_pixels_near_center(self, width, height, center_coords, radius):
        """Finds all pixels within a radius of the center coordinates.

        Args:
            width: The width of the image.
            height: The height of the image.
            center_coords: A tuple (x, y) representing the center coordinates.
            radius: The radius of the circle.

        Returns:
            A list of tuples (x, y) representing the coordinates of the pixels within
            the radius of the center coordinates.
        """

        # Calculate the top left and bottom right corners of the bounding box,
        # ensuring they stay within the image boundaries.
        top_left = (
            max(0, center_coords[0] - radius),
            max(0, center_coords[1] - radius),
        )
        bottom_right = (
            min(width - 1, center_coords[0] + radius),
            min(height - 1, center_coords[1] + radius),
        )

        # Initialize an empty list to store the coordinates of the pixels within the bounding box
        pixels_within_bounding_box = []

        # Iterate over the pixels within the bounding box, checking if they are within the radius of the center
        for y in range(top_left[1], bottom_right[1] + 1):
            for x in range(top_left[0], bottom_right[0] + 1):
                # Calculate the distance from the pixel to the center
                distance = (
                    (x - center_coords[0]) ** 2 + (y - center_coords[1]) ** 2
                ) ** 0.5
                # Add the pixel's coordinates to the list if it is within the radius and within the image bounds
                if distance <= radius and 0 <= x < width and 0 <= y < height:
                    pixels_within_bounding_box.append((x, y))

        return pixels_within_bounding_box

    def to_3d_point(
        self, img_shape, depth_map, depth_width, camera_info, radius, points
    ):
        """Calculates the 3d positions of the input predictions.

        Args:
            img_shape: shape of the image. tuple (width, height)
            depth_map:  depth_map that will be used for 3d position calculations
            depth_width:  width of the depth map
            camera_info: information need for the calculation. (f_x, f_y, c_x, c_y)
            radius: Radius around pixel that will be used for depth calculation.
            points: pixel coordinates (x, y)

        Returns:
            3d coordinates [x, y, z]
        """

        f_x = camera_info[0]
        f_y = camera_info[1]
        c_x = camera_info[2]
        c_y = camera_info[3]

        u, v = points[0], points[1]

        pixel_cords = self.find_pixels_near_center(
            img_shape[0], img_shape[1], (u, v), radius
        )

        total_depth = 0.0
        for cords in pixel_cords:
            # linear index of the pixel
            lin_idx = cords[0] + depth_width * cords[1]
            # add the depth of the pixel to the total
            if not np.isnan(depth_map[lin_idx]):
                total_depth += depth_map[lin_idx]

        z = total_depth / len(pixel_cords)
        x = ((u - c_x) * z) / (f_x)
        y = ((v - c_y) * z) / (f_y)

        return [x, y, z]

    def detect(self):
        # check if need info to run detection is received or not.
        if (
            self.received_camera_info == False
            or self.received_depth_image == False
            or self.received_rgb_image == False
        ):
            return

        try:
            # store the current_detection_target in a variable to aviod changing it while running the detection
            current_detection_target = self.current_detection_target

            input_img = self.rgb_img
            input_depth_map = self.depth_map

            im_width = self.rgb_img_width
            im_height = self.rgb_img_height
            depth_width = self.depth_map_width

            # zed camera info
            f_x = self.f_x
            f_y = self.f_y
            c_x = self.c_x
            c_y = self.c_y

            # run inference based on current target
            if current_detection_target == "landing_pad":
                img, results = self.landing_trt.infer(input_img=input_img)
                # loop over detection results and update the tracker
                for result in results:
                    predictions = result[0]
                    norfair_detections = self.yolo_to_norfair(predictions)
                    tracked_objects = self.landing_tracker.update(norfair_detections)
                    for obj in tracked_objects:
                        # flatten the list and convert the values to int
                        cord_2d = obj.get_estimate().flatten().astype(int)
                        cord_3d = self.to_3d_point(
                            img_shape=(im_width, im_height),
                            depth_map=input_depth_map,
                            depth_width=depth_width,
                            camera_info=(f_x, f_y, c_x, c_y),
                            radius=10,
                            points=cord_2d,
                        )
                        print(cord_3d)

            if current_detection_target == "avocado":
                img, results = self.avo_trt.infer(input_img=input_img)
                # WORK IN PROGRESS #

        except Exception as e:
            # cleanup
            self.landing_trt.destroy()
            self.avo_trt.destroy()
            print(e)


"""
1. Object detection model finds bounding boxes.

2. Bounding boxes are then passed to norfair tracker. 

3. Norfair tracker tracks the targets position over a 2d plane.

4. 2d pixel coordinates are used with a depth image to find that objects 3d position.

5. 3d position data is stored within PoseHistory class with timestamps.
    unique id of the TrackedObject class from norfair are used to keep track of objects.
    
6. then this data is matched with drones position at the time of detection to find the abs position of the target.     

NOTE:when the target leaves the frame and enters it again a new objects is created.
    maybe compare position data and match them?
"""


class PoseHistory(object):
    def __init__(self, object_id, obj_rel_pos, timestamp, main_object_pos):
        self._id = object_id
        self._rel_pos_list = [
            obj_rel_pos,
        ]
        self._timestamp_list = [
            timestamp,
        ]
        self._main_object_pos = [
            main_object_pos,
        ]

    @property
    def id(self):
        return self._id

    @property
    def length(self):
        return len(self._rel_pos_list)

    def new_entry(self, timestamp, obj_rel_pos, main_object_pos):
        self._rel_pos_list.append(obj_rel_pos)
        self._timestamp_list.append(timestamp)
        self._main_object_pos.append(main_object_pos)


class yolov8_trt(object):
    # initiate the engine and setup stuff
    def __init__(self, engine_file_path, input_size, output_shape):
        self.ctx = cuda.Device(0).make_context()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)
        stream = cuda.Stream()

        # deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())

        self.context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        # allocate memory for input and output
        for binding in engine:
            # create page-locked memory buffers and allocate memory on host and device
            size = trt.volume(engine.get_tensor_shape(binding))
            host_mem = cuda.pagelocked_empty(size, np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)

            # append the device buffer to device bindings
            bindings.append(int(cuda_mem))
            # append the binding the the input or output list
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
        """
        Runs inference.
        Input: Image to run Inference on
        Output: Image that was used , detection results.
                bounding boxes are in xyxy format.
        ---------------
        To loop over detections
         for result in results:
                predictions = result[0]
                for prediction in predictions:
                    bbox = prediction[:4]
                    conf = prediction[4]
                    cls = prediction[5]
        ---------------
        """

        # push the context to the gpu
        self.ctx.push()

        # set the values
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

        # preprocess
        input_img, _, _, _ = self.preprocess_image(
            input_img_raw, input_size[0], input_size[1]
        )
        # copy input image to host buffer
        np.copyto(host_inputs[0], input_img.ravel())
        # start the counter
        start = time.time()
        # transfer the input data to gpu for execution
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # run inference
        context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
        # transfer predicitions from the gpu
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        stream.synchronize()

        # end the counter
        end = time.time()

        # remove the context from the gpu
        self.ctx.pop()

        # amount of time spent
        time_spent = str(end - start)
        output = host_outputs[0]

        final_output = input_img_raw

        output = output.reshape(output_shape[0], output_shape[1], output_shape[2])
        results = self.postprocess(
            preds=output,
            img=input_img,
            orig_img=input_img_raw,
            OBJ_THRESH=0.5,
            NMS_THRESH=0.3,
        )

        return final_output, results

    def destroy(self):
        # remove the context from the gpu
        self.ctx.pop()

    def preprocess_image(self, raw_bgr_image, input_h, input_w):
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
            gain = min(
                img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1]
            )  # gain  = old / new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (
                img1_shape[0] - img0_shape[0] * gain
            ) / 2  # wh padding
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
        return 1.0 / (1 + np.exp(-x))

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

            if order.size == 1:
                break
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
        assert (
            0 <= conf_thres <= 1
        ), f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
        assert (
            0 <= iou_thres <= 1
        ), f"Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0"
        # print(prediction.shape)
        # 【lulu】prediction.shape[1]：box + cls + num_masks
        bs = prediction.shape[0]  # batch size
        nc = nc or (prediction.shape[1] - 4)  # number of classes
        nm = prediction.shape[1] - nc - 4  # num_masks
        mi = 4 + nc  # mask start index
        xc = np.max(prediction[:, 4:mi], axis=1) > conf_thres  ## 【lulu】

        # Settings
        # min_wh = 2  # (pixels) minimum box width and height
        max_wh = 7680  # (pixels) maximum box width and height
        max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
        time_limit = 0.5 + 0.05 * bs  # seconds to quit after
        redundant = True  # require redundant detections
        multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
        merge = False  # use merge-NMS

        t = time.time()
        output = [np.zeros((0, 6 + nm))] * bs  ## 【lulu】

        for xi, x in enumerate(prediction):  # image_3c index, image_3c inference
            # Apply constraints
            # x[((x[:, 2:4] < min_wh) | (x[:, 2:4] > max_wh)).any(1), 4] = 0  # width-height

            x = np.transpose(x, [1, 0])
            x = x[xc[xi]]  ## 【lulu】

            # If none remain process next image_3c
            if not x.shape[0]:
                continue

            # Detections matrix nx6 (xyxy, conf, cls)
            box, cls, mask = np.split(x, [4, 4 + nc], axis=1)  ## 【lulu】
            box = self.xywh2xyxy(
                box
            )  # center_x, center_y, width, height) to (x1, y1, x2, y2)

            j = np.argmax(cls, axis=1)  ## 【lulu】
            conf = cls[np.array(range(j.shape[0])), j].reshape(-1, 1)
            x = np.concatenate([box, conf, j.reshape(-1, 1), mask], axis=1)[
                conf.reshape(
                    -1,
                )
                > conf_thres
            ]

            # Check shape
            n = x.shape[0]  # number of boxes
            if not n:
                continue
            x = x[
                np.argsort(x[:, 4])[::-1][:max_nms]
            ]  # sort by confidence and remove excess boxes 【lulu】

            # Batched NMS
            c = x[:, 5:6] * max_wh  # classes
            boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
            i = self.nms(boxes, scores, iou_thres)  ## 【lulu】
            i = i[:max_det]  # limit detections

            output[xi] = x[i]
            if (time.time() - t) > time_limit:
                # LOGGER.warning(f'WARNING ⚠️ NMS time limit {time_limit:.3f}s exceeded')
                break  # time limit exceeded
        return output

    def letterbox(
        self,
        im,
        new_shape=(640, 640),
        color=(114, 114, 114),
        auto=True,
        scaleFill=False,
        scaleup=True,
        stride=32,
    ):
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
            ratio = (
                new_shape[1] / shape[1],
                new_shape[0] / shape[0],
            )  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(
            im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
        )  # add border
        return im, ratio, (dw, dh)

    def postprocess(self, preds, img, orig_img, OBJ_THRESH, NMS_THRESH, classes=None):
        p = self.non_max_suppression(
            preds,
            OBJ_THRESH,
            NMS_THRESH,
            agnostic=False,
            max_det=300,
            nc=classes,
            classes=None,
        )
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
            a = list(map(int, np.random.choice(range(255), 3)))
            if np.sum(a) == 0:
                continue
            color_list.append(a)
            if len(color_list) == class_num:
                break
        return color_list


def main(args=None):
    rclpy.init(args=args)

    info_node = TensorrtNode()

    rclpy.spin(info_node)

    cv2.destroyAllWindows()
    info_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()