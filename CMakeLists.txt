cmake_minimum_required(VERSION 3.8)
project(perception_asr_stressoverflow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  vision_msgs
  darknet_ros_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
  pcl_conversions
  pcl_ros
  image_geometry
  depth_image_proc
  tf2_geometry_msgs
  visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/perception_asr_stressoverflow/HSVFilterNode.cpp
  src/perception_asr_stressoverflow/DarknetDetectionNode.cpp
  src/perception_asr_stressoverflow/DetectionTo3DfromPCNode.cpp
  src/perception_asr_stressoverflow/DetectionTo3DfromDepthNode.cpp
  src/perception_asr_stressoverflow/tf_perception_asrNode.cpp
  src/perception_asr_stressoverflow/ObstacleMonitorNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(hsv_filter src/hsv_filter_main.cpp)
ament_target_dependencies(hsv_filter ${dependencies})
target_link_libraries(hsv_filter ${PROJECT_NAME})

add_executable(darknet_detection src/darknet_detection_main.cpp)
ament_target_dependencies(darknet_detection ${dependencies})
target_link_libraries(darknet_detection ${PROJECT_NAME})

add_executable(detection_2d_to_3d_pc2 src/detection_2d_to_3d_pc2_main.cpp)
ament_target_dependencies(detection_2d_to_3d_pc2 ${dependencies})
target_link_libraries(detection_2d_to_3d_pc2 ${PROJECT_NAME})

add_executable(detection_2d_to_3d_depth src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(detection_2d_to_3d_depth ${dependencies})
target_link_libraries(detection_2d_to_3d_depth ${PROJECT_NAME})

add_executable(tf_perception_asr src/tf_perception_asr_main.cpp)
ament_target_dependencies(tf_perception_asr ${dependencies})
target_link_libraries(tf_perception_asr ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  hsv_filter
  darknet_detection
  detection_2d_to_3d_pc2
  detection_2d_to_3d_depth
  tf_perception_asr
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
