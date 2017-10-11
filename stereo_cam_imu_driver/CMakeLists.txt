cmake_minimum_required(VERSION 2.8.3)
project(stereo_camera)
set(CMAKE_CXX_FLAGS 
    "${SSE_FLAGS} -O3 -g -fopenmp -march=native -std=c++11 -fPIC -fno-omit-frame-pointer -msse -msse2 -msse3 -mssse3 -msse4  -mfpmath=sse " 
    ) 
find_package(catkin REQUIRED COMPONENTS camera_info_manager image_transport nodelet roscpp sensor_msgs message_generation cv_bridge)

find_package(Boost REQUIRED COMPONENTS thread system)
#find_package(libcyusb)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_GLIB QUIET glib-2.0)

FIND_LIBRARY(
	GLIB_LIBRARIES
	NAMES glib-2.0
	HINTS ${PC_GLIB_LIBDIR}
	${PC_GLIB_LIBRARY_DIRS}
)

# Files in glib's main include path may include glibconfig.h, which,
# for some odd reason, is normally in $LIBDIR/glib-2.0/include.
GET_FILENAME_COMPONENT(_GLIB_LIBRARY_DIR ${GLIB_LIBRARIES} PATH)
FIND_PATH(
	GLIBCONFIG_INCLUDE_DIR
	NAMES glibconfig.h
	HINTS ${PC_LIBDIR} ${PC_LIBRARY_DIRS} ${_GLIB_LIBRARY_DIR}
	PATH_SUFFIXES glib-2.0/include
)

FIND_PATH(
	GLIB_INCLUDE_DIR
	NAMES glib.h
	HINTS ${PC_GLIB_INCLUDEDIR}
	${PC_GLIB_INCLUDE_DIRS}
	PATH_SUFFIXES glib-2.0
)

SET(GLIB_INCLUDE_DIRS ${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

#add_message_files(
#  FILES
#  Float64Stamped.msg
#)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
	${GLIB_INCLUDE_DIRS}
)


add_library(capture
  src/cdatacapture.h
  src/cdatacapture.cpp
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES temp
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#add_library(nodelet_uvc_camera src/nodelets.cpp src/camera.cpp src/stereo.cpp src/uvc_cam.cpp)
#target_link_libraries(nodelet_uvc_camera ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

#add_executable(uvc_camera_node src/camera_node.cpp src/camera.cpp src/uvc_cam.cpp)
#target_link_libraries(uvc_camera_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit xunitTara -lv4l2 -ludev -lglib-2.0)

#add_executable(uvc_stereo_node src/stereo_node.cpp src/stereo.cpp src/uvc_cam.cpp)
#target_link_libraries(uvc_stereo_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit xunitTara -lv4l2 -ludev -lglib-2.0)

add_executable(stereo src/stereo.cpp)
target_link_libraries(stereo ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GLIB2_LIBRARIES} capture  ${PROJECT_SOURCE_DIR}/dep/libcyusb.so -lglib-2.0)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS nodelet_uvc_camera uvc_camera_node uvc_stereo_node
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#message('haha')
#message('${CATKIN_PACKAGE_INCLUDE_DESTINATION}')
## Mark cpp header files for installation


#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uvc_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
