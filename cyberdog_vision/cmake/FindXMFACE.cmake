#  Try to find libXMFACE
#  Once done this will define
#  XMFACE_FOUND - system has libXMFACE
#  XMFACE_INCLUDE_DIRS - the libXMFACE include directory
#  XMFACE_LIBRARIES - libXMFACE library

include(FindPackageHandleStandardArgs)

find_path(XmFace_INCLUDE_DIR XMFaceAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/face_recognition/include)
find_library(XmFace_LIBRARIE libXMFaceAPI.so ${CMAKE_SOURCE_DIR}/3rdparty/face_recognition/lib)

find_package_handle_standard_args(XMFACE REQUIRED_VARS XmFace_INCLUDE_DIR XmFace_LIBRARIE)

if(XMFACE_FOUND)
    set(XMFACE_INCLUDE_DIRS ${XmFace_INCLUDE_DIR})
    set(XMFACE_LIBRARIES ${XmFace_LIBRARIE})
elseif(XMFACE_FOUND)
    message(FATAL_ERROR "Not found XMFACE")
endif()

mark_as_advanced(
    XMFACE_INCLUDE_DIRS
    XMFACE_LIBRARIES
)