#  Try to find libXmKeypoints
#  Once done this will define
#
#  XmKeypoints_FOUND - system has libXmKeypoints
#  XmKeypoints_INCLUDE_DIRS - the libXmKeypoints include directory
#  XmKeypoints_LIBRARIES - libXmKeypoints library

find_path(XmKeypoints_INCLUDE_DIRS person_keypoints.h ${CMAKE_SOURCE_DIR}/3rdparty/keypoints_detection/include)
find_library(XmKeypoints_LIBRARIES libperson_keypoints.so ${CMAKE_SOURCE_DIR}/3rdparty/keypoints_detection/lib)

if (XmKeypoints_INCLUDE_DIRS AND XmKeypoints_LIBRARIES)
    set(XmKeypoints_FOUND TRUE)
endif (XmKeypoints_INCLUDE_DIRS AND XmKeypoints_LIBRARIES)


if (XmKeypoints_FOUND)
    message(STATUS "Found XmKeypoints ${XmKeypoints_INCLUDE_DIRS} ${XmKeypoints_LIBRARIES}")
elseif (XmKeypoints_FOUND)
    message(FATAL_ERROR "Not found XmKeypoints")
endif (XmKeypoints_FOUND)