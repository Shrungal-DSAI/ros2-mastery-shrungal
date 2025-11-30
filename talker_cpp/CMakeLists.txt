cmake_minimum_required(VERSION 3.5)
project(talker_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker_cpp_node src/talker_cpp.cpp)
ament_target_dependencies(talker_cpp_node rclcpp std_msgs)

install(TARGETS
  talker_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
