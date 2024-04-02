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
        
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_sub = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)
        
        self.timer_ = self.create_timer(0.1, self.run_keyboard_control)
        
        self.path = None
        
        self.index = 0
        astar = Astar()
        if astar.plan():
            print("Path Found")
            self.path = astar.path_to_goal()
        
            
        
    def odom_callback(self, msg):
        # print("Odom Callback")
        self.x= msg.pose.position.x
        self.y= msg.pose.position.y
        orientation = msg.pose.orientation
        _,_,self.yaw = self.get_euler_from_quaternion(orientation)
        
        
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
        
    def scale_vector(self,vector, magnitude):
        # Calculate the current magnitude of the vector
        current_magnitude = np.linalg.norm(vector)
        
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


    def reached_waypoint(self, robot, current_target,threshold=1):
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

    def run_keyboard_control(self):
                
       
            current_target = self.get_next_target()
            
            robot_x, robot_y, robot_yaw = self.get_robot_oientation()
            
            print("Robot: ",robot_x, robot_y, robot_yaw)
            print("Target: ",current_target)
            if self.reached_waypoint((robot_x, robot_y), current_target):
                self.index += 1
            
            
            way_x,way_y = self.absolute2relative(current_target[0], current_target[1], robot_x, robot_y, robot_yaw)
            
            
            target_vector = self.scale_vector(np.array([way_x,way_y]),10)
            
            # Calculate the relative coordinates of the target point
            linear,angular = self.convert_to_velocity(target_vector)
            print(linear,angular)
            self.publsih_velocity(float(linear),float(angular))
            
            

                
                

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
