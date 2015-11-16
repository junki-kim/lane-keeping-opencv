cmake_minimum_required (VERSION 2.8) 

PROJECT(LaneDetect)
SET(PROJECT_VERSION "0.1")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}/home/pi/RaspiCam/lane/WiringPi-Cpp/ext/cmake)
SET(raspicam_DIR /home/pi/RaspiCam/raspicam-0.1.3/build/)
#SET(wiringPi_DIR /home/pi/Downloads/wiringPi/wiringPi/)
# Opencv Package
FIND_PACKAGE( OpenCV REQUIRED)
FIND_PACKAGE( raspicam REQUIRED)
FIND_PACKAGE( WiringPi REQUIRED)
FIND_PACKAGE( Threads REQUIRED)
include_directories(${WIRINGPI_INCLUDE_DIRS})
#link_directories(/home/pi/Downloads/wiringPi/devLib/)


IF (${OpenCV_VERSION} VERSION_LESS 2.3.0)
	MESSAGE(FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}")
ENDIF()

SET(SRC
 LaneDetect.cpp
)

SET(PROJECT_NAME
 contours
)
ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
#add_definitions(-std=c++0x -L/usr/local/lib/ -lwiringPi -lpthread)
#ADD_LIBRARY(wiringPi /home/pi/Downloads/wiringPi/wiringPi/)

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} ${raspicam_CV_LIBS} ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})



