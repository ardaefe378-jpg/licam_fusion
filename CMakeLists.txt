cmake_minimum_required(VERSION 3.8)
project(licam_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(fusion_node src/fusion_node.cpp)


target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(fusion_node
  ${OpenCV_LIBRARIES}
)


ament_target_dependencies(fusion_node
  rclcpp
  message_filters
  vision_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)


install(TARGETS
  fusion_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()