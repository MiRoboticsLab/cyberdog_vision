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

#  Try to find libXMGESTURE
#  Once done this will define
#  XMGESTURE_FOUND - system has libXMGESTURE
#  XMGESTURE_INCLUDE_DIRS - the libXMGESTURE include directory
#  XMGESTURE_LIBRARIES - libXMGESTURE library

include(FindPackageHandleStandardArgs)

find_path(XmGesture_INCLUDE_DIR hand_gesture.h ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/include)
find_library(XmGesture_LIBRARIE libhand_gesture.so ${CMAKE_SOURCE_DIR}/3rdparty/body_gesture/lib)

find_package_handle_standard_args(XMGESTURE REQUIRED_VARS XmGesture_INCLUDE_DIR XmGesture_LIBRARIE)

if(XMGESTURE_FOUND)
    set(XMGESTURE_INCLUDE_DIRS ${XmGesture_INCLUDE_DIR})
    set(XMGESTURE_LIBRARIES ${XmGesture_LIBRARIE})
elseif(XMGESTURE_FOUND)
    message(FATAL_ERROR "Not found XMGESTURE")
endif()

mark_as_advanced(
    XMGESTURE_INCLUDE_DIRS
    XMGESTURE_LIBRARIES
)