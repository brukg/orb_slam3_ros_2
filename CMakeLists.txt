cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /home/phoenix/ros2_ws/src/open-source/opencv/build)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(
  include 
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  DWoB2
  Sophus
)


set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/lib/libORB_SLAM3.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)


add_executable(mono_orb_slam_node 
  src/nodes/mono_orb_slam_node.cpp 
)

ament_target_dependencies(mono_orb_slam_node 
  rclcpp 
  sensor_msgs
  OpenCV
  cv_bridge
)
set_target_properties(mono_orb_slam_node PROPERTIES CXX_STANDARD 17)


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


target_link_libraries(mono_orb_slam_node
  ${LIBS}
)

install(TARGETS
mono_orb_slam_node
DESTINATION lib/${PROJECT_NAME})
ament_package()

install(DIRECTORY 
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)
