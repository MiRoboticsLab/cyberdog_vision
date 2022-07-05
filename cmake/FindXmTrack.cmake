#  Try to find libXmTrack
#  Once done this will define
#
#  XmTrack_FOUND - system has libXmTrack
#  XmTrack_INCLUDE_DIRS - the libXmTrack include directory
#  XmTrack_LIBRARIES - libXmTrack library

find_path(XmTrack_INCLUDE_DIRS tracker.hpp ${CMAKE_SOURCE_DIR}/3rdparty/auto_track/include)
find_library(XmTrack_LIBRARIES libtracker.so ${CMAKE_SOURCE_DIR}/3rdparty/auto_track/lib)

if (XmTrack_INCLUDE_DIRS AND XmTrack_LIBRARIES)
    set(XmTrack_FOUND TRUE)
endif (XmTrack_INCLUDE_DIRS AND XmTrack_LIBRARIES)


if (XmTrack_FOUND)
    message(STATUS "Found XmTrack ${XmTrack_INCLUDE_DIRS} ${XmTrack_LIBRARIES}")
elseif (XmTrack_FOUND)
    message(FATAL_ERROR "Not found XmTrack")
endif (XmTrack_FOUND)