cmake_minimum_required(VERSION 3.8)
project(grasping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(executable_list
  publish_goal_pose_joy
)

foreach(loop_var IN LISTS executable_list)
  add_executable(${loop_var} src/${loop_var}.cpp)
  ament_target_dependencies(${loop_var}
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_msgs
    moveit_ros_planning_interface
  )

  install(TARGETS
    ${loop_var}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
