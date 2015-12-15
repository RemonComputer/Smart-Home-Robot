cmake_minimum_required(VERSION 2.8.11)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

project(Interface)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#set(${OPENNI_INCLUDE_DIR} "D:/Program Files/OpenNI/Include")
#set(${OPENNI_LIBRARY} "D:/Program Files/OpenNI/Lib64/openNI64.lib")
#set(${Qt5Core_DIR} "D:/Qt/5.0.2/msvc2012_64/lib/cmake/Qt5Core")
#set(${Qt5Concurrent_DIR} "D:/Qt/5.0.2/msvc2012_64/lib/cmake/Qt5Concurrent")


# Tell CMake to create the helloworld executable
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(Qt5Concurrent REQUIRED)
find_package(OpenCV REQUIRED )
#find_package(OPENNI REQUIRED )
FIND_PACKAGE( MRPT REQUIRED base hwdrivers slam vision gui obs)    # WARNING: Add all the MRPT libs used by the program: "gui", "obs", "slam",etc.

#INCLUDE(${QT_USE_FILE})

include_directories(${OPENCV_INCLUDE_DIR})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Concurrent_INCLUDE_DIRS})
#include_directories(${OPENNI_INCLUDE_DIR})
include_directories("D:/Program Files/OpenNI/Include")

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} /MP")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


add_executable (Interface main.cpp SharedResource.h SharedResource.cpp OdometryGrabber.h  OdometryGrabber.cpp KinectGrabber.h KinectGrabber.cpp
Explorer.h Explorer.cpp Learner.h Learner.cpp MicroCommander.h MicroCommander.cpp MyObject.h MyObject.cpp SLAM.cpp 
SLAM.h Viewer.h Viewer.cpp PathDirector.h PathDirector.cpp Segmentor.h Segmentor.cpp)
target_link_libraries (Interface   ${OpenCV_LIBS} ${MRPT_LIBS} Qt5::Core Qt5::Concurrent "D:/Program Files/OpenNI/Lib64/openNI64.lib")

# Set optimized building: for MRPT
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")