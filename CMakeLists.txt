##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.16.3)
project(qtros)

# debug mdoe.
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(CMAKE_CXX_STANDARD 14)
# This makes C++14 a requirement,
# and prevents a "decay" to C++98 when the compiler does not support C++14.
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
# This disables the use of compiler-specific extensions.
# For example, by default, CMake passes -std=gnu++14 to GCC on Linux.
# We want to build with -std=c++14. 
SET(CMAKE_CXX_EXTENSIONS OFF)

# Don't change the values of CMAKE_CXX_FLAGS,
# as those are mainly intended to be changed through toolchain files or the cache.
SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")  # ikd-Tree

#add
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

ADD_DEFINITIONS(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
  livox_ros_driver
  message_generation
)
include_directories(${catkin_INCLUDE_DIRS})
#add
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)
SET(PCL_DIR /usr/local/share/pcl-1.13)
FIND_PACKAGE(PCL 1.13 REQUIRED)
SET(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.13)
add_definitions(${PCL_DEFINITIONS})
MESSAGE(STATUS "PCL version: "${PCL_VERSION})
FIND_PACKAGE(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
FIND_PATH(IKD_INCLUDE_DIR NAMES ikd_Tree.h PATHS /home/neal/usr/include/ikd_Tree)
FIND_LIBRARY(IKD_LIBRARIES ikd_Tree HINTS /home/neal/usr/lib)
FIND_PATH(LOGGER_INCLUDE_DIR NAMES file_logger.h PATHS /home/neal/usr/include/file_logger)
FIND_LIBRARY(LOGGER_LIBRARIES file_logger HINTS /home/neal/usr/lib)

link_libraries("/home/neal/projects/qt_ros/src/qtros/lib/libcontrolcan.so")

ADD_MESSAGE_FILES(
  FILES
  Pose6D.msg
)

GENERATE_MESSAGES(
  DEPENDENCIES
  geometry_msgs
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
CATKIN_PACKAGE(
  INCLUDE_DIRS include include/lio include/qtros
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs tf livox_ros_driver message_generation
  DEPENDS EIGEN3 PCL
)

INCLUDE_DIRECTORIES(
  include
  include/lio
  include/qtros
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${IKD_INCLUDE_DIR}
  ${LOGGER_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIRS}
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#remove
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qtros/*.h include/lio/*.h)

#change
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(ros_qt_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ros_qt_demo ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${IKD_LIBRARIES} ${LOGGER_LIBRARIES} ${ZLIB_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS ros_qt_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

