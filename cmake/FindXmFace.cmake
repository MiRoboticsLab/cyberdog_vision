#  Try to find libXmFace
#  Once done this will define
#
#  XmFace_FOUND - system has libXmFace
#  XmFace_INCLUDE_DIRS - the libXmFace include directory
#  XmFace_LIBRARIES - libXmFace library

find_path(XmFace_INCLUDE_DIRS XMFaceAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/face_recognition/include)
find_library(XmFace_LIBRARIES libXMFaceAPI.so ${CMAKE_SOURCE_DIR}/3rdparty/face_recognition/lib)

if (XmFace_INCLUDE_DIRS AND XmFace_LIBRARIES)
    set(XmFace_FOUND TRUE)
endif (XmFace_INCLUDE_DIRS AND XmFace_LIBRARIES)


if (XmFace_FOUND)
    message(STATUS "Found XmFace ${XmFace_INCLUDE_DIRS} ${XmFace_LIBRARIES}")
elseif (XmFace_FOUND)
    message(FATAL_ERROR "Not found XmFace")
endif (XmFace_FOUND)