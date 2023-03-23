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

#  Try to find libXMKEYPOINTS
#  Once done this will define
#  XMKEYPOINTS_FOUND - system has libXMKEYPOINTS
#  XMKEYPOINTS_INCLUDE_DIRS - the libXMKEYPOINTS include directory
#  XMKEYPOINTS_LIBRARIES - libXMKEYPOINTS library

include(FindPackageHandleStandardArgs)

find_path(XmKeypoints_INCLUDE_DIR person_keypoints.h ${CMAKE_SOURCE_DIR}/3rdparty/keypoints_detection/include)
find_library(XmKeypoints_LIBRARIE libperson_keypoints.so ${CMAKE_SOURCE_DIR}/3rdparty/keypoints_detection/lib)

find_package_handle_standard_args(XMKEYPOINTS REQUIRED_VARS XmKeypoints_INCLUDE_DIR XmKeypoints_LIBRARIE)

if(XMKEYPOINTS_FOUND)
    set(XMKEYPOINTS_INCLUDE_DIRS ${XmKeypoints_INCLUDE_DIR})
    set(XMKEYPOINTS_LIBRARIES ${XmKeypoints_LIBRARIE})
elseif(XMKEYPOINTS_FOUND)
    message(FATAL_ERROR "Not found XMKEYPOINTS")
endif()

mark_as_advanced(
    XMKEYPOINTS_INCLUDE_DIRS
    XMKEYPOINTS_LIBRARIES
)