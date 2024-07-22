cmake_minimum_required(VERSION 3.8)
project(realtime_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# Topic Communication Example
# Publisher 
add_executable(realtime_publisher_node src/realtime_publisher_node.cpp)
ament_target_dependencies(realtime_publisher_node rclcpp std_msgs)

# Subscriber
add_executable(realtime_subscriber_node src/realtime_subscriber_node.cpp)
ament_target_dependencies(realtime_subscriber_node rclcpp std_msgs)

# Service Communication Example
# Server
add_executable(realtime_service_server_node src/realtime_service_server_node.cpp)
ament_target_dependencies(realtime_service_server_node rclcpp example_interfaces)

# Client
add_executable(realtime_service_client_node src/realtime_service_client_node.cpp)
ament_target_dependencies(realtime_service_client_node rclcpp example_interfaces)


install(TARGETS
  realtime_publisher_node
  realtime_subscriber_node
  realtime_service_server_node
  realtime_service_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
