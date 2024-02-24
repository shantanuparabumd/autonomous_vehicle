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


#include "autonomous_vehicle/car.hpp" //<-- Include the car header file
#include <string>


//=====================================
Car::Car(std::string car_name, std::string node_name)
      : Node(node_name),
        m_car_name{car_name} /// Initialize the Car name
    {

    m_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Set QoS profile
    rclcpp::QoS qos_profile(10);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    qos_profile.keep_last(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::SystemDefault);

    // Topic names

    // Central laser scan topic
    auto center_scan_topic_name = "center_laser/scan";  ///< Create the scan topic name
    
    // Front sonar topics
    auto front_rigth_far_topic = "front_sonar/right_far_range";  ///< Create the front right far topic name
    auto front_rigth_middle_topic = "front_sonar/right_middle_range";  ///< Create the front right middle topic name
    auto front_left_far_topic = "front_sonar/left_far_range";  ///< Create the front left far topic name
    auto front_left_middle_topic = "front_sonar/left_middle_range";  ///< Create the front left middle topic name
    
    // Front camera topics
    auto front_camera_topic_name = "front_camera_sensor/image_raw";  ///< Create the camera topic name
    auto front_processed_image  = "front_camera_sensor/processed_image";  ///< Create the image topic name

    // Velocity and steering angle topics
    auto linear_velocity_topic_name = "velocity_controller/commands";  ///< Create the linear velocity topic name
    auto steering_angle_topic_name = "position_controller/commands";  ///< Create the steering angle topic name
    
    // Velocity and steering angle publisher

    // Create a publisher for the velocity command
    m_publisher_linear_vel = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        linear_velocity_topic_name, 10);

    // Create a publisher for the steering angle command
    m_publisher_steering_angle = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        steering_angle_topic_name, 10);

    // Image publisher

    // Create a publisher for the front camera processed image
    m_front_processed_image_publisher =
        this->create_publisher<sensor_msgs::msg::Image>(front_processed_image, 10);

    
    
    // Create a subscriber for the laser scan messages
    m_center_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            center_scan_topic_name, qos_profile,
            std::bind(&Car::center_scan_callback, this, std::placeholders::_1));

    // Create a subscriber for camera messages
    m_front_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                front_camera_topic_name, qos_profile,
                std::bind(&Car::front_camera_callback, this, std::placeholders::_1));

    
    // Call on_timer function 1 times per second
    m_front_image_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Car::front_image_pub_callback, this), m_cbg);

    // Call on_timer function 1 times per second
    m_move_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Car::move, this), m_cbg);


    // Create a subscriber for the laser scan messages
    m_front_right_far_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_rigth_far_topic, qos_profile,
            std::bind(&Car::front_right_far_scan_callback, this, std::placeholders::_1));
    
    // Create a subscriber for the laser scan messages
    m_front_right_middle_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_rigth_middle_topic, qos_profile,
            std::bind(&Car::front_right_middle_scan_callback, this, std::placeholders::_1));
    
    // Create a subscriber for the laser scan messages
    m_front_left_far_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_left_far_topic, qos_profile,
            std::bind(&Car::front_left_far_scan_callback, this, std::placeholders::_1));
    
    // Create a subscriber for the laser scan messages
    m_front_left_middle_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_left_middle_topic, qos_profile,
            std::bind(&Car::front_left_middle_scan_callback, this, std::placeholders::_1));


  }


//=====================================
void Car::center_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_center_scan_data = msg.ranges;

}

void Car::front_right_far_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_front_right_far_scan_data = msg.ranges;

}

void Car::front_right_middle_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_front_right_middle_scan_data = msg.ranges;

}

void Car::front_left_far_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_front_left_far_scan_data = msg.ranges;

}

void Car::front_left_middle_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
  // Get the ranges from the LaserScan message
  m_front_left_middle_scan_data = msg.ranges;

}



//=====================================
void Car::front_camera_callback(const sensor_msgs::msg::Image &msg) {
  // Convert the ROS image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;

  // Convert the ROS image message to an OpenCV image
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }


  // Display Image Name
  cv::putText(cv_ptr->image, "Front Camera Image" ,
     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

  // Convert the OpenCV image to a ROS image message
  sensor_msgs::msg::Image::SharedPtr processed_image_msg_ptr =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();

  // Store the pointer to the processed image message to be publsihed later
  m_front_processed_image_msg = *processed_image_msg_ptr;
}


void Car::publish_control_commands(double linear, double steering)
{
  // Create a message for the linear velocity
  auto linear_velocity_msg = std_msgs::msg::Float64MultiArray();

  // Create a message for the steering angle
  auto steering_angle_msg = std_msgs::msg::Float64MultiArray();

  // Set the linear velocity
  linear_velocity_msg.data = {linear, linear, linear, linear};

  // Set the steering angle
  steering_angle_msg.data = {steering, steering};

  // Publish the linear velocity
  m_publisher_linear_vel->publish(linear_velocity_msg);

  // Publish the steering angle
  m_publisher_steering_angle->publish(steering_angle_msg);
}

void Car::front_image_pub_callback()
{
  // Publish the processed image
  m_front_processed_image_publisher->publish(m_front_processed_image_msg);
}

void Car::move(){
    publish_control_commands(4.0, 0.0);

    if (check_obstacle(m_front_right_far_scan_data, 2.0, 0, 2) == true) {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected on the right side");
      publish_control_commands(0.0, 0.0);
    }
    else {
      publish_control_commands(4.0, 0.0);
    }
}


//=====================================
bool Car::check_obstacle(const std::vector<float> scan_data, float threshold,int center, int range) {
  // Flag to store if the obstacle is detected
  bool obstacle_detected = false;

  // Calculate the low and high range of angles to consider
  int low = center - range;
  int high = center + range;

  
  // Iterate through the range of angles and check for obstacles
  for (int i = low; i <= high; ++i) {
    // For positive angles
    if (i >= 0 && i < static_cast<int>(scan_data.size())) {
      // Check if the distance is less than the threshold

      if (scan_data[i] > 0.0 && scan_data[i]  < threshold) {
        RCLCPP_INFO_STREAM(this->get_logger(),"Scan Data: "<< scan_data[0]);
        // If the distance is less than the threshold,
        // set the obstacle_detected flag to true and break
        obstacle_detected = true;
        break;
      }
    }
    // For negative angles convert the negative index to positive index
    if (i < 0) {
      // Check if the distance is less than the threshold
      
      if (scan_data[static_cast<int>(scan_data.size()) + i] > 0 && scan_data[static_cast<int>(scan_data.size()) + i] < threshold) {
        RCLCPP_INFO_STREAM(this->get_logger(),"Scan Data: "<< scan_data[0]);
        // If the distance is less than the threshold,
        // set the obstacle_detected flag to true and break
        obstacle_detected = true;
        break;
      }
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(),"Return "<< obstacle_detected);
  return obstacle_detected;
}