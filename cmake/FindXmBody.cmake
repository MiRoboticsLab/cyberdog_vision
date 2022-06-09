#  Try to find libXmBody
#  Once done this will define
#
#  XmBody_FOUND - system has libXmBody
#  XmBody_INCLUDE_DIRS - the libXmBody include directory
#  XmBody_LIBRARIES - libXmBody library

find_path(XmBody_INCLUDE_DIRS ContentMotionAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/include)
find_library(XmBody_LIBRARIES libContentMotionAPI.so ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/lib)

if (XmBody_INCLUDE_DIRS AND XmBody_LIBRARIES)
    set(XmBody_FOUND TRUE)
endif (XmBody_INCLUDE_DIRS AND XmBody_LIBRARIES)


if (XmBody_FOUND)
    message(STATUS "Found XmBody ${XmBody_INCLUDE_DIRS} ${XmBody_LIBRARIES}")
elseif (XmBody_FOUND)
    message(FATAL_ERROR "Not found XmBody")
endif (XmBody_FOUND)