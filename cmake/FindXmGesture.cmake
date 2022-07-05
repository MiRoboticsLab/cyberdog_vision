#  Try to find libXmGesture
#  Once done this will define
#
#  XmGesture_FOUND - system has libXmGesture
#  XmGesture_INCLUDE_DIRS - the libXmGesture include directory
#  XmGesture_LIBRARIES - libXmGesture library

find_path(XmGesture_INCLUDE_DIRS hand_gesture.h ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/include)
find_library(XmGesture_LIBRARIES libhand_gesture.so ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/lib)

if (XmGesture_INCLUDE_DIRS AND XmGesture_LIBRARIES)
    set(XmGesture_FOUND TRUE)
endif (XmGesture_INCLUDE_DIRS AND XmGesture_LIBRARIES)


if (XmGesture_FOUND)
    message(STATUS "Found XmGesture ${XmGesture_INCLUDE_DIRS} ${XmGesture_LIBRARIES}")
elseif (XmGesture_FOUND)
    message(FATAL_ERROR "Not found XmGesture")
endif (XmGesture_FOUND)