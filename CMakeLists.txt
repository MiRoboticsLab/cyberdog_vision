cmake_minimum_required(VERSION 3.8)
project(cyberdog_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.2/)

# third_party_library
set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR})
set(THIRD_PARTY_URL https://cnbj2m-fds.api.xiaomi.net/bsp-internal/ROS/carpo-camera/AI/3rdparty.zip)
execute_process(
  COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/3rdparty
  COMMAND wget -q ${THIRD_PARTY_URL} -P ${CMAKE_SOURCE_DIR}/3rdparty
)
execute_process(
  COMMAND unzip ${CMAKE_SOURCE_DIR}/3rdparty/3rdparty.zip
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty
)
execute_process(COMMAND rm ${CMAKE_SOURCE_DIR}/3rdparty/3rdparty.zip)
add_subdirectory(3rdparty)

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(XmBody REQUIRED)
find_package(XmFace REQUIRED)
find_package(XmGesture REQUIRED)
find_package(XmKeypoints REQUIRED)
find_package(XmReID REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(vision_manager
  src/main.cpp
  src/vision_manager.cpp
  src/body_detection.cpp
  src/face_recognition.cpp
  src/gesture_recognition.cpp
  src/keypoints_detection.cpp
  src/person_reid.cpp
)

add_executable(test_ai
  src/test_ai.cpp
  src/body_detection.cpp
  src/face_recognition.cpp
  src/gesture_recognition.cpp
  src/keypoints_detection.cpp
  src/person_reid.cpp
)

ament_target_dependencies(vision_manager rclcpp OpenCV CUDA XmBody XmFace XmGesture XmKeypoints XmReID)
ament_target_dependencies(test_ai rclcpp OpenCV CUDA XmBody XmFace XmGesture XmKeypoints XmReID)

install(TARGETS
  vision_manager
  test_ai
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
