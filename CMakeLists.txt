cmake_minimum_required (VERSION 2.8) 

PROJECT(LaneDetect)
SET(PROJECT_VERSION "0.1")
SET(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH)/usr/local/lib/cmake/)      
# Opencv Package
FIND_PACKAGE( OpenCV REQUIRED)
FIND_PACKAGE( raspicam REQUIRED)
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

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} ${raspicam_CV_LIBS})



