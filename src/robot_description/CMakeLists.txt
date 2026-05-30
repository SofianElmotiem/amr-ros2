cmake_minimum_required(VERSION 3.5)
project(robot_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Omni-wheel Gazebo plugin
add_library(omni_wheel_plugin SHARED src/omni_wheel_plugin.cpp)

target_include_directories(omni_wheel_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(omni_wheel_plugin
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(omni_wheel_plugin
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS omni_wheel_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY robot meshes config launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
