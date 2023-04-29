cmake_minimum_required(VERSION 3.5)
project(soccer_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "srv/Kill.srv"
  "srv/Spawn.srv"
  "srv/TeleportAbsolute.srv"
  )

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")

set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/simulation_frame.cpp
)
set(turtlesim_node_HDRS
  include/soccer_sim/simulation_frame.h
)

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(turtlesim_node Qt5::Widgets)
ament_target_dependencies(turtlesim_node ${dependencies})
target_link_libraries(turtlesim_node "${cpp_typesupport_target}")

install(TARGETS turtlesim_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

ament_package()
