cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /home/phoenix/ros2_ws/src/open-source/opencv/build)
# add_subdirectory(thirdparty/Sophus)
# add_subdirectory(thirdparty/DBoW2)
# add_subdirectory(thirdparty/g2o)
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
  src/orb_slam3/System.cc
)

ament_target_dependencies(mono_orb_slam_node 
  rclcpp 
  sensor_msgs
  OpenCV
  cv_bridge
)


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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(ORB_SLAM3 SHARED
src/orb_slam3/System.cc
src/orb_slam3/Tracking.cc
src/orb_slam3/LocalMapping.cc
src/orb_slam3/LoopClosing.cc
src/orb_slam3/ORBextractor.cc
src/orb_slam3/ORBmatcher.cc
src/orb_slam3/FrameDrawer.cc
src/orb_slam3/Converter.cc
src/orb_slam3/MapPoint.cc
src/orb_slam3/KeyFrame.cc
src/orb_slam3/Atlas.cc
src/orb_slam3/Map.cc
src/orb_slam3/MapDrawer.cc
src/orb_slam3/Optimizer.cc
src/orb_slam3/Frame.cc
src/orb_slam3/KeyFrameDatabase.cc
src/orb_slam3/Sim3Solver.cc
src/orb_slam3/Viewer.cc
src/orb_slam3/ImuTypes.cc
src/orb_slam3/G2oTypes.cc
src/orb_slam3/CameraModels/Pinhole.cpp
src/orb_slam3/CameraModels/KannalaBrandt8.cpp
src/orb_slam3/OptimizableTypes.cpp
src/orb_slam3/MLPnPsolver.cpp
src/orb_slam3/GeometricTools.cc
src/orb_slam3/TwoViewReconstruction.cc
src/orb_slam3/Config.cc
src/orb_slam3/Settings.cc
# include/orb_slam3/System.hpp
# include/orb_slam3/Tracking.h
# include/orb_slam3/LocalMapping.h
# include/orb_slam3/LoopClosing.h
# include/orb_slam3/ORBextractor.h
# include/orb_slam3/ORBmatcher.h
# include/orb_slam3/FrameDrawer.h
# include/orb_slam3/Converter.h
# include/orb_slam3/MapPoint.h
# include/orb_slam3/KeyFrame.h
# include/orb_slam3/Atlas.h
# include/orb_slam3/Map.h
# include/orb_slam3/MapDrawer.h
# include/orb_slam3/Optimizer.h
# include/orb_slam3/Frame.h
# include/orb_slam3/KeyFrameDatabase.h
# include/orb_slam3/Sim3Solver.h
# include/orb_slam3/Viewer.h
# include/orb_slam3/ImuTypes.h
# include/orb_slam3/G2oTypes.h
# include/orb_slam3/CameraModels/GeometricCamera.h
# include/orb_slam3/CameraModels/Pinhole.h
# include/orb_slam3/CameraModels/KannalaBrandt8.h
# include/orb_slam3/OptimizableTypes.h
# include/orb_slam3/MLPnPsolver.h
# include/orb_slam3/GeometricTools.h
# include/orb_slam3/TwoViewReconstruction.h
# include/orb_slam3/SerializationUtils.h
# include/orb_slam3/Config.h
# include/orb_slam3/Settings.h
)

target_link_libraries(ORB_SLAM3
  # include
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/lib/libg2o.so
  -lboost_serialization
  -lcrypto
)