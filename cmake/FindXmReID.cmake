#  Try to find libXmReID
#  Once done this will define
#
#  XmReID_FOUND - system has libXmReID
#  XmReID_INCLUDE_DIRS - the libXmReID include directory
#  XmReID_LIBRARIES - libXmReID library

find_path(XmReID_INCLUDE_DIRS ReIDToolAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/person_reid/include)
find_library(XmReID_LIBRARIES libReIDTools.so ${CMAKE_SOURCE_DIR}/3rdparty/person_reid/lib)

if (XmReID_INCLUDE_DIRS AND XmReID_LIBRARIES)
    set(XmReID_FOUND TRUE)
endif (XmReID_INCLUDE_DIRS AND XmReID_LIBRARIES)


if (XmReID_FOUND)
    message(STATUS "Found XmReID ${XmReID_INCLUDE_DIRS} ${XmReID_LIBRARIES}")
elseif (XmReID_FOUND)
    message(FATAL_ERROR "Not found XmReID")
endif (XmReID_FOUND)