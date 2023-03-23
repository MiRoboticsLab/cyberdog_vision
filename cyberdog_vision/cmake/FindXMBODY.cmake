#  Try to find libXMBODY
#  Once done this will define
#  XMBODY_FOUND - system has libXMBODY
#  XMBODY_INCLUDE_DIRS - the libXMBODY include directory
#  XMBODY_LIBRARIES - libXMBODY library

include(FindPackageHandleStandardArgs)

find_path(XmBody_INCLUDE_DIR ContentMotionAPI.h ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/include)
find_library(XmBody_LIBRARIE libContentMotionAPI.so ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/lib)

find_package_handle_standard_args(XMBODY DEFAULT_MSG XmBody_INCLUDE_DIR XmBody_LIBRARIE)

if(XMBODY_FOUND)
    set(XMBODY_INCLUDE_DIRS ${XmBody_INCLUDE_DIR})
    set(XMBODY_LIBRARIES ${XmBody_LIBRARIE})
elseif(XMBODY_FOUND)
    message(FATAL_ERROR "Not found XMBODY")
endif()

mark_as_advanced(
    XMBODY_INCLUDE_DIRS
    XMBODY_LIBRARIES
)