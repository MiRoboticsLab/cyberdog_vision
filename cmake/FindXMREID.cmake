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