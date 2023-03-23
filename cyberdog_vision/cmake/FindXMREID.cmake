#  Try to find libXMREID
#  Once done this will define
#  XMREID_FOUND - system has libXMREID
#  XMREID_INCLUDE_DIRS - the libXMREID include directory
#  XMREID_LIBRARIES - libXMREID library

include(FindPackageHandleStandardArgs)

find_path(XmReID_INCLUDE_DIR ReIDToolAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/person_reid/include)
find_library(XmReID_LIBRARIE libReIDTools.so ${CMAKE_SOURCE_DIR}/3rdparty/person_reid/lib)

find_package_handle_standard_args(XMREID REQUIRED_VARS XmReID_INCLUDE_DIR XmReID_LIBRARIE)

if(XMREID_FOUND)
    set(XMREID_INCLUDE_DIRS ${XmReID_INCLUDE_DIR})
    set(XMREID_LIBRARIES ${XmReID_LIBRARIE})
elseif(XMREID_FOUND)
    message(FATAL_ERROR "Not found XMREID")
endif()

mark_as_advanced(
    XMREID_INCLUDE_DIRS
    XMREID_LIBRARIES
)