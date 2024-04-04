#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from autonomous_vehicle.astar import Astar

import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan,Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2
import random
from cv_bridge import CvBridge, CvBridgeError


import math

# Define key codes
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.max_linear_speed = 10
        self.max_angular_speed = 10
        
        self.lidar_msg = None
        
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_sub = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)
        
        self.bridge = CvBridge()
        # Define the QoS profile
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Keep only the last message
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        
        self.lidar_sub = self.create_subscription(LaserScan, '/center_laser/scan', self.lidar_callback, qos_profile)
        self.timer_ = self.create_timer(0.1, self.run_keyboard_control)
        
        self.image_sub = self.create_subscription(Image, 'front_camera_sensor/image_raw', self.image_callback, qos_profile)
        
        self.image_pub = self.create_publisher(Image,'processed_image',10)
        
        self.path = [(1038, 475)]
        
        self.cv_image = np.zeros((800, 800, 3), np.uint8)
        
        self.side_walk_repulsion = np.array([0,0])
        
        self.index = 0
        astar = Astar()
        if astar.plan():
            print("Path Found")
            self.path = astar.path_to_goal()
        
    def image_callback(self, msg):
        # print("Odom Callback")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            # print(f"Line: {(x1+x2)//2} x: {x} y: {y}")
            cv2.arrowedLine(canvas, (400,400), (x,y), (0, 255, 0), 2)
        
        x_component =  np.cos(0) / result[0]
        y_component =  np.sin(0) / result[0]
        self.side_walk_repulsion = -1*self.scale_vector([x_component,y_component],10)
        # print(f"Result: {result}")
        # cv2.arrowedLine(canvas, (400,400), (int(result[0]+400),400), (0, 0, 255), 2)
        # cv2.imshow("Vector Image", canvas)
        # cv2.waitKey(1)
        
    def new_plan(self):
        astar = Astar()
        if astar.plan():
            print("Path Found")
            self.path = astar.path_to_goal()
            self.index = 0
        else:
            print("No Path Found")
            self.path = None
        
    def odom_callback(self, msg):
        # print("Odom Callback")
        self.x= msg.pose.position.x
        self.y= msg.pose.position.y
        orientation = msg.pose.orientation
        _,_,self.yaw = self.get_euler_from_quaternion(orientation)
        
    def lidar_callback(self, msg):
        # print("Lidar Callback")
        self.lidar_msg = msg
        
    def parse_laser_data (self,laser_data):
        if laser_data is None:
            dist = [10]*512
        else:
            dist = laser_data.ranges
            
        angles = np.linspace(-3.14, 3.14, 512)
        laser = [(min(dist[i],10), angles[i]) for i in range(len(dist))]
        
        return laser
    
    def laser_data_to_repulsive_vectors(self,laser_data):
        repulsive_vectors = []
        for distance, angle in laser_data:
            if angle >= -np.pi / 2 and angle <= np.pi / 2:
                if distance==0:
                    distance=0.01
                # Calculate x and y components of the vector
                x_component =  np.cos(angle) / distance
                y_component =  np.sin(angle) / distance
                repulsive_vectors.append([x_component, y_component])
        # Sum all repulsive vectors to get the resultant vector
        resultant_vector = -1* np.sum(repulsive_vectors, axis=0)
        
        return resultant_vector/5
        
        
        
    def get_euler_from_quaternion(self, quaternion):
        # Normalize the quaternion
        norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm
        quaternion.w /= norm

        # Convert quaternion to Euler angles
        roll = math.atan2(2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1 - 2 * (quaternion.x**2 + quaternion.y**2))
        pitch = math.asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x))
        yaw = math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y**2 + quaternion.z**2))

        return roll, pitch, yaw

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
            
    def publsih_velocity(self, linear_vel, steer_angle):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        # Publish the twist message
        wheel_velocities.data = [linear_vel,linear_vel,linear_vel,linear_vel]
        joint_positions.data = [steer_angle,steer_angle]

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
    def convert_to_velocity(self,resultant_vector):
        # Calculate linear velocity magnitude
        linear_velocity_magnitude = np.linalg.norm(resultant_vector)
        # Normalize the resultant vector to get its direction
        direction = resultant_vector / linear_velocity_magnitude if linear_velocity_magnitude > 0 else np.array([0, 0])

        # Calculate angular velocity magnitude based on the direction angle
        angle = np.arctan2(direction[1], direction[0])
        angular_velocity_magnitude = angle * self.max_angular_speed / np.pi

        # Scale linear velocity within the maximum speed limit
        linear_velocity =  min(linear_velocity_magnitude, self.max_linear_speed)

        return linear_velocity, angular_velocity_magnitude
        
    def scale_vector(self, vector, magnitude):
        # Calculate the current magnitude of the vector
        current_magnitude = np.linalg.norm(vector)
        
        if current_magnitude == 0:
            # Handle the case where the vector is already at the origin (0, 0)
            return np.zeros_like(vector)
        
        # Scale the vector to the desired magnitude
        scaled_vector = vector * (magnitude / current_magnitude)
        
        return scaled_vector
    
    
    def map_value(self,value, in_min, in_max, out_min, out_max):
        # Map a value from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def get_next_target(self):
        y,x= self.path[self.index]
        x=1724-x
        y = self.map_value(y,0,2076,103.8,-103.8)
        x = self.map_value(x,0,1724,-48.7,123.7)
        return (x,y)
    
    def get_robot_oientation(self):
        return self.x, self.y, self.yaw


    def reached_waypoint(self, robot, current_target,threshold=7):
        # Calculate the distance between the robot and the target point
        distance = np.sqrt((robot[0] - current_target[0])**2 + (robot[1] - current_target[1])**2)
        
        # Check if the robot has reached the target point
        if distance < threshold:
            return True
        
        return False
    
    def absolute2relative (self,x_abs, y_abs, robotx, roboty, robott):

        # robotx, roboty are the absolute coordinates of the robot
        # robott is its absolute orientation
        # Convert to relatives
        dx = x_abs - robotx
        dy = y_abs - roboty

        # Rotate with current angle
        x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
        y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

        return x_rel , y_rel
    
    def process_vector(self, v):
        theta = np.radians(90)

        # Compute the rotated vector
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        print("Initial Vector: ",v  )
        vector = np.array([cos_theta * v[0] - sin_theta * v[1],
                            sin_theta * v[0] + cos_theta * v[1]])
        print("Vector: ",vector)
        x,y = int(vector[0]*10), int(vector[1]*10)
        x = 320 + x
        y = 240 - y
        end_point = (x, y)
        return end_point
        
    def show_vector(self, vector1, vector2, vector3,target):
        # Define the dimensions of the canvas
        height, width = 480, 640

        # Create a blank canvas filled with white color
        blank_canvas = np.ones((height, width, 3), np.uint8) * 255
    
        green = (0, 255, 0)  # Green color for vectors
        red = (0, 0, 255)  # Red color for vectors
        black = (0, 0, 0)  # Black color for vectors
        
        thickness = 2
        
        start_point = (320, 240)
        # print(F"Target: {target}")
        # print("Target: ",self.process_vector(target))
        cv2.circle(blank_canvas, self.process_vector(target), 5, (0, 0, 0), -1)
        
        cv2.arrowedLine(blank_canvas, start_point, self.process_vector(vector1), red, thickness)
        cv2.arrowedLine(blank_canvas, start_point, self.process_vector(vector2), green, thickness)
        cv2.arrowedLine(blank_canvas, start_point, self.process_vector(vector3), black, thickness)
        cv2.imshow('Image with Vectors', blank_canvas)
        cv2.waitKey(1)
        # Add a delay to control the frame rate
        # time.sleep(0.1)

        # Check if the window is closed
        # if cv2.getWindowProperty('Image', cv2.WND_PROP_VISIBLE) < 1:
        #     break
        
        
        
    
    def run_keyboard_control(self):
                
       
            current_target = self.get_next_target()
            
            # current_target = (0,0)
            
            robot_x, robot_y, robot_yaw = self.get_robot_oientation()
            
            # print("Robot: ",robot_x, robot_y, robot_yaw)
            # print("Target: ",current_target)
            if self.reached_waypoint((robot_x, robot_y), current_target):
                self.index += 1
                if self.index >= len(self.path):
                    self.publsih_velocity(0.0,0.0)
                    self.new_plan()
            
            
            way_x,way_y = self.absolute2relative(current_target[0], current_target[1], robot_x, robot_y, robot_yaw)
            
            
            target_vector = self.scale_vector(np.array([way_x,way_y]),10)
        
            laser  = self.parse_laser_data(self.lidar_msg)
            
            # for x,y in laser:
            #     if x<5:
            #         print(f"Distance: {x} Angle: {math.degrees(y)}")
            
            repulsive_vector = self.laser_data_to_repulsive_vectors(laser) + self.side_walk_repulsion
            
            # print("Path: ",self.path[self.index])
            print("Current Target: ",current_target)
            print("Robot: ",robot_x, robot_y, robot_yaw)
            
            direction = target_vector + repulsive_vector
            
            self.show_vector(repulsive_vector,target_vector,direction,(way_x,way_y))
            
            # Calculate the relative coordinates of the target point
            linear,angular = self.convert_to_velocity(direction)
            print(linear,angular)
            self.publsih_velocity(float(linear),float(angular))
            
            

                
                

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publsih_velocity(0.0,0.0)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.publsih_velocity(0.0,0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
