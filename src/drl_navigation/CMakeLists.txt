cmake_minimum_required(VERSION 3.8)
project(drl_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR .
)

install(FILES
  launch_sim.launch.py
  rsp.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config
  assets
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  train.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
