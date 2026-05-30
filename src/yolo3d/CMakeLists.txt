cmake_minimum_required(VERSION 3.8)
project(yolo3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/inference_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/utils/activations.py
  scripts/utils/augmentations.py
  scripts/utils/autoanchor.py
  scripts/utils/autobatch.py
  scripts/utils/callbacks.py
  scripts/utils/datasets.py
  scripts/utils/downloads.py
  scripts/utils/general.py
  scripts/utils/__init__.py
  scripts/utils/loss.py
  scripts/utils/metrics.py
  scripts/utils/plots.py
  scripts/utils/torch_utils.py
  DESTINATION lib/${PROJECT_NAME}/utils
)

install(
  PROGRAMS
  scripts/models/common.py
  scripts/models/experimental.py
  scripts/models/__init__.py
  scripts/models/tf.py
  scripts/models/yolo.py
  scripts/models/yolov5l.yaml
  scripts/models/yolov5m.yaml
  scripts/models/yolov5n.yaml
  scripts/models/yolov5s.yaml
  scripts/models/yolov5x.yaml
  DESTINATION lib/${PROJECT_NAME}/models
)

install(
  PROGRAMS
  scripts/script/ClassAverages.py
  scripts/script/Dataset.py
  scripts/script/Dataset_lightning.py
  scripts/script/__init__.py
  scripts/script/Model.py
  scripts/script/Model_lightning.py
  scripts/script/class_averages.txt
  DESTINATION lib/${PROJECT_NAME}/script
)

install(
  PROGRAMS
  scripts/library/Calib.py
  scripts/library/__init__.py
  scripts/library/Math.py
  scripts/library/Plotting.py
  DESTINATION lib/${PROJECT_NAME}/library
)

install(
  PROGRAMS
  scripts/data/coco.yaml
  scripts/data/coco128.yaml
  scripts/data/VOC.yaml
  DESTINATION lib/${PROJECT_NAME}/data
)

install(
  PROGRAMS
  scripts/eval/camera_cal/calib_cam_to_cam.txt
  DESTINATION lib/${PROJECT_NAME}/eval/camera_cal
)

ament_package()
