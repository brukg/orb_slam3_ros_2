cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)



set(OpenCV_DIR /home/phoenix/ros2_ws/src/open-source/opencv/build)


# find dependencies
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(
  ../include 
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  DWoB2
  Sophus
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../lib)

add_library(${PROJECT_NAME} SHARED
../src/orb_slam3/System.cc
../src/orb_slam3/Tracking.cc
../src/orb_slam3/LocalMapping.cc
../src/orb_slam3/LoopClosing.cc
../src/orb_slam3/ORBextractor.cc
../src/orb_slam3/ORBmatcher.cc
../src/orb_slam3/FrameDrawer.cc
../src/orb_slam3/Converter.cc
../src/orb_slam3/MapPoint.cc
../src/orb_slam3/KeyFrame.cc
../src/orb_slam3/Atlas.cc
../src/orb_slam3/Map.cc
../src/orb_slam3/MapDrawer.cc
../src/orb_slam3/Optimizer.cc
../src/orb_slam3/Frame.cc
../src/orb_slam3/KeyFrameDatabase.cc
../src/orb_slam3/Sim3Solver.cc
../src/orb_slam3/Viewer.cc
../src/orb_slam3/ImuTypes.cc
../src/orb_slam3/G2oTypes.cc
../src/orb_slam3/CameraModels/Pinhole.cpp
../src/orb_slam3/CameraModels/KannalaBrandt8.cpp
../src/orb_slam3/OptimizableTypes.cpp
../src/orb_slam3/MLPnPsolver.cpp
../src/orb_slam3/GeometricTools.cc
../src/orb_slam3/TwoViewReconstruction.cc
../src/orb_slam3/Config.cc
../src/orb_slam3/Settings.cc
../include/orb_slam3/System.hpp
../include/orb_slam3/Tracking.h
../include/orb_slam3/LocalMapping.h
../include/orb_slam3/LoopClosing.h
../include/orb_slam3/ORBextractor.h
../include/orb_slam3/ORBmatcher.h
../include/orb_slam3/FrameDrawer.h
../include/orb_slam3/Converter.h
../include/orb_slam3/MapPoint.h
../include/orb_slam3/KeyFrame.h
../include/orb_slam3/Atlas.h
../include/orb_slam3/Map.h
../include/orb_slam3/MapDrawer.h
../include/orb_slam3/Optimizer.h
../include/orb_slam3/Frame.h
../include/orb_slam3/KeyFrameDatabase.h
../include/orb_slam3/Sim3Solver.h
../include/orb_slam3/Viewer.h
../include/orb_slam3/ImuTypes.h
../include/orb_slam3/G2oTypes.h
../include/orb_slam3/CameraModels/GeometricCamera.h
../include/orb_slam3/CameraModels/Pinhole.h
../include/orb_slam3/CameraModels/KannalaBrandt8.h
../include/orb_slam3/OptimizableTypes.h
../include/orb_slam3/MLPnPsolver.h
../include/orb_slam3/GeometricTools.h
../include/orb_slam3/TwoViewReconstruction.h
../include/orb_slam3/SerializationUtils.h
../include/orb_slam3/Config.h
../include/orb_slam3/Settings.h
)

target_link_libraries(${PROJECT_NAME}
  # include
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../lib/libg2o.so
  -lboost_serialization
  -lcrypto
)