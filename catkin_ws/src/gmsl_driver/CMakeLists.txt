cmake_minimum_required(VERSION 3.1)
project(gmsl_n_cameras)

option(DEFINE_ENABLE_TCP_SOCKET
    "Enable TCP Socket Server" OFF)     # Disabled by default

if(DEFINE_ENABLE_TCP_SOCKET)
	set(DEFINE_TCP_SERVER 1)
else(DEFINE_ENABLE_TCP_SOCKET)
	set(DEFINE_TCP_SERVER 0)
endif(DEFINE_ENABLE_TCP_SOCKET)

set (CMAKE_CXX_STANDARD 11)

if(UNIX)
    set(PATH_SEPARATOR         '/')
else(UNIX)
    set(PATH_SEPARATOR         '\\')
endif(UNIX)

# FindDriveworks.cmake, ArchConfiguration.cmake, and LibFindMacros.cmake were needed for my setup they are taken from driveworks/samples/cmake/
# ArchConfiguration.cmake was the only file that needed small changes, remove the fatal error on line 17 and add the following lines in its place
         set(VIBRANTE TRUE)
         add_definitions(-DVIBRANTE)
# this is the path I placed the driveworks cmake files in
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(Driveworks REQUIRED)
find_package(OpenCV REQUIRED )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/inConfig/buildType.h.in
    ${CMAKE_CURRENT_SOURCE_DIR}/src/buildType.h @ONLY)

# SET THESE VARIABLES TO YOUR DRIVEWORKS LOCATIONS
set(Driveworks_INCLUDE_DIR /usr/local/driveworks/include/)
set(Driveworks_LIBRARY /usr/local/driveworks/lib/)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr/local)

if(NOT DEFINE_ENABLE_TCP_SOCKET) 
	message(STATUS "Building ROS Support")
	find_package(catkin REQUIRED COMPONENTS
  		roscpp
  		geometry_msgs
  		cv_bridge 
  		image_transport
	)

	catkin_package(
  		LIBRARIES ${PROJECT_NAME}
	)
else(NOT DEFINE_ENABLE_TCP_SOCKET)
	message(STATUS "Building TCP Support")
endif(NOT DEFINE_ENABLE_TCP_SOCKET)

link_directories(
  ${Driveworks_LIBRARY})

MESSAGE("Driveworks include directory ${Driveworks_INCLUDE_DIR}")

if(NOT DEFINE_ENABLE_TCP_SOCKET)
	include_directories( ${catkin_INCLUDE_DIRS} )
	# ros node name template
	set(NODE_NAME ${PROJECT_NAME}_node)
else(NOT DEFINE_ENABLE_TCP_SOCKET)
	set(NODE_NAME ${PROJECT_NAME})
endif(NOT DEFINE_ENABLE_TCP_SOCKET)

include_directories(
  ${Driveworks_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# TODO: add a FindNvmedia.cmake file for this? Why does it not exist?
include_directories(/usr/share/visionworks/sources/3rdparty/nvmedia/
/usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/include/
)

if(DEFINE_ENABLE_TCP_SOCKET)
  set(TCP_SRC src/tcpSocket.cpp src/tcpSocketPrivate.cpp)
endif(DEFINE_ENABLE_TCP_SOCKET)

add_executable(${NODE_NAME} src/main.cpp
  src/ProgramArguments.cpp
  src/ConsoleColor.cpp
  src/cv_connection.cpp
  src/ResourceManager.cpp
  src/Camera.cpp
  ${TCP_SRC}
)

if(NOT DEFINE_ENABLE_TCP_SOCKET)
	set(ROS_LIBS ${catkin_LIBRARIES})
endif(NOT DEFINE_ENABLE_TCP_SOCKET)

target_link_libraries(${NODE_NAME}
  ${ROS_LIBS}
  nvmedia  
  driveworks
  GLESv2
  EGL
  drm
  ${CUDA_LIBRARY}
  ${OpenCV_LIBS} 
  ${CMAKE_THREAD_LIBS_INIT}
)
