cmake_minimum_required(VERSION 3.8)
project(autonomous_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake_python REQUIRED)


include_directories(include)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

install(
  DIRECTORY include/autonomous_vehicle
  DESTINATION include
)


add_executable(car_control src/main.cpp)
ament_target_dependencies(car_control tf2 tf2_ros rclcpp cv_bridge std_msgs sensor_msgs geometry_msgs nav_msgs)

add_library(vehicle_library src/car.cpp)
target_include_directories(vehicle_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
ament_target_dependencies(vehicle_library tf2 tf2_ros  cv_bridge rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)





target_link_directories(car_control PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(car_control vehicle_library)

install(TARGETS
  car_control
  DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
  scripts/key_board.py
  scripts/waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

# if(COVERAGE)
#   set(CMAKE_BUILD_TYPE "Debug")
#   set(CMAKE_CXX_FLAGS "--coverage")
#   set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
#   set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg"
#     CACHE STRING "uncrustify config file")
# endif()

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)

  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  # set(TESTFILES
  #   test/test.cpp)
  # ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  # ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs  geometry_msgs)
  # target_link_libraries(test_${PROJECT_NAME} vehicle_library)
  # install(TARGETS
  #   test_${PROJECT_NAME}
  #   DESTINATION lib/${PROJECT_NAME})
endif()

# install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
#   scripts/generate_coverage_report.bash
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
