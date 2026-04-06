
project(basler_blaze_fusion LANGUAGES CXX)
cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  camera_info_manager
  #cv_bridge
  #pcl_ros
  message_generation
)

# Locate the cmake find module for pylon ...
find_path(pylon_cmake Findpylon.cmake 
    HINTS "$ENV{PYLON_DEV_DIR}/lib/cmake"
          "$ENV{ProgramFiles}/Basler/pylon 6/Development/lib/cmake"
          "$ENV{ProgramFiles}/Basler/pylon 5/Development/lib/cmake"
          "/opt/pylon/lib/cmake"
        )
if (NOT pylon_cmake )
  message(FATAL_ERROR 
  "Findpylon.cmake module not found. Either set the PYLON_DEV_DIR"
      "environment variable or the pylon_ROOT_DIR CMake variable to the location "
      "the Basler pylon SDK is installed in.")
endif()
# ... and extend the cmake module path accordingly.
list(APPEND CMAKE_MODULE_PATH ${pylon_cmake})

#find_package(pylon 6.2 REQUIRED)
if (NOT pylon_FOUND)
    find_package(pylon 6.2 REQUIRED)
endif()

if (NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif()

if (NOT PCL_FOUND)
    find_package(PCL QUIET REQUIRED COMPONENTS common visualization filters)
endif()

## Generate services in the 'srv' folder.


## Generate messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin-specific configuration ##
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs camera_info_manager cv_bridge pcl_ros message_generation
)

## Build ##
include_directories(/usr/include/pcl-1.10/ )
#include_directories(/usr/include/vtk-7.1/ )
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/fusion_node.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pylon::pylon
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES}
)

string(STRIP "${PCL_DEFINITIONS}" PCL_DEFINITIONS)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE ${PCL_DEFINITIONS} CALIBRATION_DIR="${CMAKE_CURRENT_SOURCE_DIR}/CalibrationData/")
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)


