/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2024 Shantanu Parab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#ifndef INCLUDE_AUTONOMOUS_VEHICLE_CAR_HPP_
#define INCLUDE_AUTONOMOUS_VEHICLE_CAR_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"  //<-- Include the rclcpp library
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/image.hpp>  // Included for Image message
#include <sensor_msgs/msg/laser_scan.hpp>   // Included for LaserScan message
#include "cv_bridge/cv_bridge.h"  // Included for cv_bridge (OPENCV TO ROS)
#include <opencv2/highgui.hpp>  // Included for highgui  (OPENCV)
#include <string>

class Car: public rclcpp::Node{ //<-- Inherit the class from rclcpp
    public:

        /**
         * @brief Construct a new Car object
         * 
         * The constructor initializes the car node with a name and node name.
         * 
         * @param robot_name 
         * @param node_name 
         */
        Car(std::string car_name, std::string node_name);



        /**
        * @brief Read the front camera data from the robot and process it
        * 
        * The robot reads the camera data and processes it to detect the red color box
        * It computes the angle and depth of the centroid of the box and the intensity of the box
        * It also computes the x and y coordinates of the box and stores it in the objective_location variable
        * 
        * @param msg 
        */
        void front_camera_callback(const sensor_msgs::msg::Image &msg);

        /**
        * @brief Callback method for the LiDAR scan.
        * 
        * The robot checks for obstacles in the LiDAR scan and if an obstacle is detected,
        * takes the necessary action to avoid the obstacle.
        *
        * @param msg 
        */
        void center_scan_callback(const sensor_msgs::msg::LaserScan &msg);

        /**
         * @brief  Callback method for the front right far sonar scan.
         *
         * The robot checks for obstacles in the front right far sonar scan and if an obstacle is detected,
         * 
         * @param msg 
         */
        void front_right_far_scan_callback(const sensor_msgs::msg::LaserScan &msg);

        
        /**
         * @brief  Callback method for the front right middle sonar scan.
         *
         * The robot checks for obstacles in the front right middle sonar scan and if an obstacle is detected,
         * 
         * @param msg 
         */
        void front_right_middle_scan_callback(const sensor_msgs::msg::LaserScan &msg);

        /**
         * @brief  Callback method for the front left far sonar scan.
         *
         * The robot checks for obstacles in the front left far sonar scan and if an obstacle is detected,
         * 
         * @param msg 
         */
        void front_left_far_scan_callback(const sensor_msgs::msg::LaserScan &msg);

        /**
         * @brief  Callback method for the front left middle sonar scan.
         *
         * The robot checks for obstacles in the front left middle sonar scan and if an obstacle is detected,
         * 
         * @param msg 
         */
        void front_left_middle_scan_callback(const sensor_msgs::msg::LaserScan &msg);
        
        /**
        * @brief To move to robot by publish velocities on "linear velocity" and "steering angle" topics
        *
        * @param linear linear velocity component
        * @param steering Steer angle
        */
        void publish_control_commands(double linear, double steering);


        /**
        * @brief Publishes the image message
        * 
        * Publishes the processed image message to the topic "front_camera/processed_image".
        * This is used to display the processed image in the GUI
        */
        void front_image_pub_callback();

        /**
         * @brief Call back function for the control loop
         * 
         */
        void move();


        bool check_obstacle(const std::vector<float> scan_data, float threshold,int center, int range);
            
    private:


        rclcpp::CallbackGroup::SharedPtr m_cbg;

        // Publishers

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_linear_vel;  ///< Publisher for the velocity command

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_steering_angle;  ///< Publisher for the steering angle command

        // Image Publishers

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_front_processed_image_publisher;  ///< The publisher object for processed image.


        // Image Subscriber

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_front_camera_subscriber_;  ///< The subscriber object for camera data.

        
        // LiDAR Subscriber
        
        // Create a subscriber for the laser scan messages
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_center_scan_subscriber_;  ///< The subscriber object for LiDAR data.

        // Front sonar subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_front_right_far_scan_subscriber_;  ///< The subscriber object for front right far sonar data.

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_front_right_middle_scan_subscriber_;  ///< The subscriber object for front right middle sonar data.
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_front_left_far_scan_subscriber_;  ///< The subscriber object for front left far sonar data.

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_front_left_middle_scan_subscriber_;  ///< The subscriber object for front left middle sonar data.


        // Timers

        rclcpp::TimerBase::SharedPtr m_move_timer;  ///< Timer for the control loop


        rclcpp::TimerBase::SharedPtr m_front_image_timer;  ///< Timer to trigger image publishing callback.
        
        // Variables

        std::string m_car_name;  ///< The name of the car

        std::vector<float> m_center_scan_data;  ///< The LiDAR data from the center LiDAR sensor
        
        std::vector<float> m_front_right_far_scan_data;  ///< The LiDAR data from the front right far LiDAR sensor

        std::vector<float> m_front_right_middle_scan_data;  ///< The LiDAR data from the front right middle LiDAR sensor

        std::vector<float> m_front_left_far_scan_data;  ///< The LiDAR data from the front left far LiDAR sensor

        std::vector<float> m_front_left_middle_scan_data;  ///< The LiDAR data from the front left middle LiDAR sensor

        sensor_msgs::msg::Image m_front_processed_image_msg; ///< The processed image message for front camera
        
};

#endif // INCLUDE_AUTONOMOUS_VEHICLE_CAR_HPP_
