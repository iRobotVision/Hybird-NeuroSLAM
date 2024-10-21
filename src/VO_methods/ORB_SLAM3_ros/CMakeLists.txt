cmake_minimum_required(VERSION 2.8.3)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(ORBSLAM_DIR "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
#set(CMAKE_CXX_STANDARD 17)
#add_definitions(-DCOMPILEDWITHC17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


find_package(catkin REQUIRED COMPONENTS
        message_generation std_msgs roscpp
        sensor_msgs nav_msgs tf visualization_msgs
        image_transport nav_msgs cv_bridge)

# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行
find_package(OpenCV 4.2 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.2 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        geometry_msgs std_msgs roscpp
        tf sensor_msgs image_transport
)
include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}

${ORBSLAM_DIR}
${ORBSLAM_DIR}/include
${ORBSLAM_DIR}/include/CameraModels
${ORBSLAM_DIR}/Thirdparty/Sophus

${Pangolin_INCLUDE_DIRS}
${YAML_CPP_INCLUDE_DIR}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORBSLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORBSLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORBSLAM_DIR}/lib/libORB_SLAM3.so
-lboost_system

${catkin_LIBRARIES}
${YAML_CPP_LIBRARIES}
)



add_executable(Stereo_pub
        src/ros_stereo_pub.cpp)
target_link_libraries(Stereo_pub
        ${LIBS} ${catkin_LIBRARIES})

add_executable(Stereo_Inertial_pub src/ros_stereo_inertial_pub.cpp)
target_link_libraries(Stereo_Inertial_pub ${LIBS})
