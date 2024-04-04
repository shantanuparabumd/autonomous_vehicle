#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2



class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        
        # Define the QoS profile
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Keep only the last message
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        
        self.image_sub = self.create_subscription(Image, 'front_camera_sensor/image_raw', self.image_callback, qos_profile)
        
        self.image_pub = self.create_publisher(
            Image,
            'processed_image',
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cv_image = np.zeros((480, 640, 3), np.uint8)
            
   
        
    def image_callback(self, msg):
        # print("Odom Callback")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        
    def timer_callback(self):
        
        self.process_image(self.cv_image)
        # Publish the image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        
    def process_image(self, cv_image):
        # Process the image
        cv2.putText(cv_image, "Processed Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        lane_lines,sidewalk_lines = self.extract_lane_sidewalk_lines(cv_image)
        # Draw the detected lane markings on the original image
        canvas_size = (800, 800)
        canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)*255
        result = np.array([0.0, 0.0])
        for line in lane_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            x= (x1+x2)//2
            y=400
            result += np.array([x-400,0])
            print(f"Line: {(x1+x2)//2} x: {x} y: {y}")
            cv2.arrowedLine(canvas, (400,400), (x,y), (0, 255, 0), 2)
        print(f"Result: {result}")
        cv2.arrowedLine(canvas, (400,400), (int(result[0]+400),400), (0, 0, 255), 2)
        cv2.imshow("Vector Image", canvas)
        cv2.waitKey(1)

        
        
    def extract_lane_sidewalk_lines(self,image):
        # Convert the image to grayscale
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform edge detection using Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Perform Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=150, maxLineGap=10)

        # Get the height of the image
        height, _ = image.shape[:2]

        # Filter out lines based on slope and position
        lane_lines = []
        sidewalk_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Calculate slope of the line
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Add a small value to avoid division by zero
            # Filter lines based on slope and y-coordinate
            if y1 > height / 2 and y2 > height / 2:  # Ensure both endpoints are below half the image height
                if 0.1 < np.abs(slope) < 2.5:  # Adjust slope thresholds as needed
                    # Lane markings are typically nearly horizontal
                    lane_lines.append(line)
                
            

        return lane_lines, sidewalk_lines
    

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
