# cyberdog_vision

## Introduction

Cyberdog_vision is a package based on ROS2. This package mainly used for AI algorithm scheduling management and AI algorithm inference. The  AI algorithms included is as follows:

- Face Recognition with facial attributes (e.g. age, emotion)
- Body Detection
- ReID
- Static Gesture Recognition
- Human Keypoints Detection
- Auto Track

## Dependencies

In addition to the basic pkgs of ros2, the external dependencies are as follows:

- cyberdog_common
- protocol

## Installation

You can install this package on Linux system as follows:

- create your workspace
- clone the code from git
- compile and install

```
colcon build --packages-up-to cyberdog_vision --install-base=/opt/ros2/cyberdog/ --merge-install
```

## Usage

The AI algorithm can be used alone in the following ways:

```
# 1. Start camera to get image stream
ros2 run camera_test camera_server

# 2. Start vision_manager node
ros2 run cyberdog_vision vision_manager

# 3. Configure node
ros2 service call /vision_manager/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

# 4. Select AI algorithm you need, take body tracking as an example
ros2 service call /algo_manager protocol/srv/AlgoManager "{algo_enable: ['algo_module':1,'algo_module':4]}"

# 5. Activate node
ros2 service call /vision_manager/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# 6. Deactivate node
ros2 service call /vision_manager/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}"

# 7. Cleanup node
ros2 service call /vision_manager/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 2}}"
```

## Node info

/vision_manager
  **Subscribers:**
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  **Publishers:**
    /facemanager/face_result: protocol/msg/FaceResult
    /person: protocol/msg/Person
    /processing_status: protocol/msg/TrackingStatus
    /vision_manager/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  **Service Servers:**
    /algo_manager: protocol/srv/AlgoManager
    /facemanager: protocol/srv/FaceManager
    /tracking_object: protocol/srv/BodyRegion
    /vision_manager/change_state: lifecycle_msgs/srv/ChangeState
    /vision_manager/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /vision_manager/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /vision_manager/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /vision_manager/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /vision_manager/get_parameters: rcl_interfaces/srv/GetParameters
    /vision_manager/get_state: lifecycle_msgs/srv/GetState
    /vision_manager/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /vision_manager/list_parameters: rcl_interfaces/srv/ListParameters
    /vision_manager/set_parameters: rcl_interfaces/srv/SetParameters
    /vision_manager/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  **Service Clients:**
    /camera_service: protocol/srv/CameraService
  **Action Servers:**

  **Action Clients:**
