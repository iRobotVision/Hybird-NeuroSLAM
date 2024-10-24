cmake_minimum_required(VERSION 2.8.3)
project(ratslam_ros)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS_RELEASE  ${CMAKE_CXX_FLAGS} -O3)

find_package(catkin REQUIRED COMPONENTS
        message_generation std_msgs roscpp
        sensor_msgs nav_msgs tf visualization_msgs
        image_transport nav_msgs
        cv_bridge)

add_message_files(
        FILES
        gaussianCells.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)


find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        src
        ${OpenCV_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        "/home/daybeha/Documents/github/Hybird-NeuroSLAM/to_release_ws/devel/include"
)
message(${OpenCV_LIBRARIES})

# dbow3
set(DBoW3_INCLUDE_DIRS "/usr/local/include/DBoW3" )
set(DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

catkin_package(
        CATKIN_DEPENDS message_runtime geometry_msgs std_msgs roscpp
)

set(LIBS
        ${catkin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${Sophus_LIBRARIES}
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)
# add vtk support
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu)

option(WITH_OPENMP "Build with OpenMP support for multithreading" ON)
# add OpenMP support
if (WITH_OPENMP)
        find_package(OpenMP QUIET)
        list(APPEND CERES_COMPILE_OPTIONS CERES_USE_OPENMP)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
        find_package(Threads REQUIRED)
        list(APPEND CERES_COMPILE_OPTIONS CERES_HAVE_PTHREAD)
        list(APPEND CERES_COMPILE_OPTIONS CERES_HAVE_RWLOCK)
        if (NOT OPENMP_FOUND)
                set(WITH_OPENMP OFF)
        endif ()
endif ()


add_library(neuroslam
        ${DBoW3_LIBS}
        src/utils/utils.cpp
        src/neuroslam/visual_memory.cpp
        src/neuroslam/posecell_network.cpp
        src/neuroslam/experience_map.cpp
        src/neuroslam/BayesCells.cpp
)

target_link_libraries(neuroslam
        ${OpenMP_LIBS}
        ${gsl_LIBRARIES}
        ${DBoW3_LIBS}
)

add_executable(ratslam_from_odom_opt src/main_from_odom_opt.cpp)
target_link_libraries(ratslam_from_odom_opt ${LIBS} neuroslam)
