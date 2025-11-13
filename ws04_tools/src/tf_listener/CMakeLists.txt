cmake_minimum_required(VERSION 3.5)
project(tf_listener)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_listener_node src/tf_listener_node.cpp)

ament_target_dependencies(tf_listener_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)
# install launch files so ros2 launch can find them
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  tf_listener_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
