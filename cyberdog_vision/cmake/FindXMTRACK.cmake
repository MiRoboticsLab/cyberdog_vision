#  Try to find libXMTRACK
#  Once done this will define
#  XMTRACK_FOUND - system has libXMTRACK
#  XMTRACK_INCLUDE_DIRS - the libXMTRACK include directory
#  XMTRACK_LIBRARIES - libXMTRACK library

include(FindPackageHandleStandardArgs)

find_path(XmTrack_INCLUDE_DIR tracker.hpp ${CMAKE_SOURCE_DIR}/3rdparty/auto_track/include)
find_library(XmTrack_LIBRARIE libtracker.so ${CMAKE_SOURCE_DIR}/3rdparty/auto_track/lib)

find_package_handle_standard_args(XMTRACK REQUIRED_VARS XmTrack_INCLUDE_DIR XmTrack_LIBRARIE)

if(XMTRACK_FOUND)
    set(XMTRACK_INCLUDE_DIRS ${XmTrack_INCLUDE_DIR})
    set(XMTRACK_LIBRARIES ${XmTrack_LIBRARIE})
elseif(XMTRACK_FOUND)
    message(FATAL_ERROR "Not found XMTRACK")
endif()

mark_as_advanced(
    XMTRACK_INCLUDE_DIRS
    XMTRACK_LIBRARIES
)