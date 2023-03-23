# Copyright (c) 2021 Xiaomi Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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