cmake_minimum_required(VERSION 3.8)
project(command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_definitions(-DBAUDRATE=B9600)

file(GLOB_RECURSE SRC_FILE src/*.cpp)

include_directories(include)
add_executable(command_node ${SRC_FILE})
ament_target_dependencies(command_node rclcpp sensor_msgs)

install(
  TARGETS 
  command_node  
  DESTINATION 
  lib/${PROJECT_NAME}
)

ament_package()
