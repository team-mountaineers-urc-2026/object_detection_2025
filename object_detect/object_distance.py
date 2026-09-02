# Subscribes to the yolo node and publishes the estimated position of the object

import pickle
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from robot_interfaces.msg import ObjectPoint
from robot_interfaces.msg import ImageMetadata
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import math
from ultralytics import YOLO
import torch
import os
from ament_index_python.packages import get_package_share_directory #connect the packages
from autonomy_2026.autonomy_targets import AutonomyTargets as Targets
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json 
import time
import threading
import subprocess
from queue import Queue

class DistanceNode(Node):
   
    class_dict = [100, 0.36, 0.36, 0.215, 0.4]  # Misc, Misc, Keyboard Height, Bottle Height

    
    def __init__(self):
        super().__init__(f'distance_node')

        self.declare_parameter("base_ips", ["192.168.1.110"])
        self.declare_parameter("base_user", "lenovo")
        
        # FOr testing saving images
        self.save_dir = "/home/jetson/workspace-deimos/src/object_detection_2025/detections"
        os.makedirs(self.save_dir, exist_ok=True)

        # For signaling detections
        self.detected_object_pub = self.create_publisher(Int32, "/detected_object_id", 10)

        self.scp_queue = Queue(maxsize=5)
        self.scp_thread = threading.Thread(target=self.scp_worker, daemon=True)
        self.scp_thread.start()

        # Execute the YOLO model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        #pkg_path = get_package_share_directory('object_detect')
        self.get_logger().info(f"device = {self.device}")
        self.config_file = os.path.join('launches', 'config', 'best.pt')

        self.model = YOLO(self.config_file)
        # NEW: Trying to insert tolerance via model.
        # Tested, does work, however, with model 'best.pt'
        # Object 3 is constantly detected in idle air, with a confidence
        # anywhere as low as 0.067 up to 0.226. 
        # Set starting Confidence threshold to 0.3 
        # as a solution to this for now. This seems a good spot as well
        # as hammer detections seem to vary well between 0.3 and 0.89
        self.confidence_threshold = 0.65#

        #NEW: params for switching to yolo-e #John Stuff
        self.prompts=[
            'blue water bottle',
            'red water bottle',
            'water bottle',
            'orange dead blow hammer',
            'yellow handled gray rock pick'
        ]
       # self.model.set_classes(self.prompts) # John Stuff

        # Topics 
        self.declare_parameter('global_origin_frame', 'base_link')
        self.declare_parameter('camera_metadata', '/CM_cam_meta')
        self.declare_parameter('cam_images', 'image_topic')
        self.declare_parameter('cam_object', '/cam_object_pose')

        # Metadata Parameters
        self.focal_length = 3.67
        self.sensor_height = 1.0
        self.image_height = 1
        self.image_width = 1


        #Setup Camera Info Subscriber-Republisher
        self.cam_images = self.get_parameter('cam_images').get_parameter_value().string_value
        self.create_subscription(
            Image,
            self.cam_images,
            self.cam_images_callback,
            5
        )

        #Gui Confidence Subscriber for changing Conf threshold on the Go.
        self.subscription = self.create_subscription(
            Float32,
            'confidence_changer', #topic_name for now...
            self.confidence_callback,
            5
        )

        # Setup Camera Metadata Subscriber
        self.camera_metadata= self.get_parameter('camera_metadata').get_parameter_value().string_value
        self.create_subscription(
            ImageMetadata,
            self.camera_metadata,
            self.cam_metadata_callback,
            5
        )

        # Publisher for Objects Location Relative to Camera
        self.distance_publisher = self.create_publisher(ObjectPoint, '/autonomy/cam_object', 5)
        #publisher for bounding box 
        self.boundingbox_pub=self.create_publisher(String, 'autonomy/bounding_boxes', 10)

        #publisher for no detections
        self.no_detect_pub = self.create_publisher(Bool, 'autonomy/no_detection', 10)

    #Confidence Changer Callback
    def confidence_callback(self, msg):
        self.confidence_threshold = msg.data
        self.get_logger().info(f"Updated confidence threshold to: {self.confidence_threshold}")
       
    def cam_metadata_callback(self, msg):
        self.focal_length = msg.foc_len_mm 
        self.sensor_height = msg.sensor_height
        self.image_height = msg.im_height
        self.image_width = msg.im_width



    def cam_images_callback(self, msg):
        # Convert ROS image message to OpenCV (BGR format)
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return
                                                                         #NEW: confidence_threshold added to yolo model (does work)
        results = self.model(cv_image, device=self.device, imgsz = 1920, conf=self.confidence_threshold)    # yolo_output: [x, y, w, h, class]
       
        result = results[0]
        # Saving individual frames of detections, to be streamed later (TEST)
        if len(result.boxes) > 0:

            for box in result.boxes:
                object_class = int(box.cls.item())

                detected_msg = Int32()

                if (object_class == 0):
                    detected_msg.data = Targets.BOTTLE.value
                elif (object_class == 1):
                    detected_msg.data = Targets.HAMMER.value
                elif (object_class ==2):
                    detected_msg.data = Targets.ROCKPICK.value
                else:
                    continue

                self.detected_object_pub.publish(detected_msg)
                self.get_logger().info(f"Published detected object ID: {detected_msg.data}")


            print("saving frame........")
            annotated_frame = result.plot()
            timestamp = time.time()*1000
            filename = os.path.join(self.save_dir, f"det_{timestamp}.jpg")

            #THIS IS WHERE WE REPLACE WITH STREAMING TO THE GUI
            print("saving too: ", filename)
            cv2.imwrite(filename, annotated_frame)

            try:
                self.scp_queue.put_nowait(filename)
            except:
                self.get_logger().warn("SCP QUEUE FULL")
        else:
            self.no_detect_pub.publish(True)

    def scp_worker(self):
        while rclpy.ok():
            filename = self.scp_queue.get()

            try:
                self.send_via_scp(filename)
            except Exception as e:
                print(f"SCP worker error: {e}")

            self.scp_queue.task_done()

    def send_via_scp(self, filename):
        """ Executes the SCP transfer """
        ips = self.get_parameter("base_ips").value
        name = self.get_parameter("base_user").value
		
        for ip in ips:
            print(f"Attempting to send to {ip}...")
            data = subprocess.run(["sshpass", "-p", name, "scp", filename, f"{name}@{ip}:/home/{name}/Desktop/Objects_Detected"])
            if data.returncode == 0:
                print(f"Successfully sent to {ip}")
            else:
                print(f"Failed to send to {ip}")


def main(args=None):
    rclpy.init(args=args)
    
    subscriber = DistanceNode()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
