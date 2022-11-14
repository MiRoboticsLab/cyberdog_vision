// Copyright (c) 2021 Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_VISION__VISION_MANAGER_HPP_
#define CYBERDOG_VISION__VISION_MANAGER_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/body.hpp"
#include "protocol/msg/face.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/face_info.hpp"
#include "protocol/msg/track_result.hpp"
#include "protocol/msg/algo_list.hpp"
#include "protocol/msg/person.hpp"
#include "protocol/msg/tracking_status.hpp"
#include "protocol/msg/face_result.hpp"
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/algo_manager.hpp"
#include "protocol/srv/face_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "cyberdog_vision/shared_memory_op.hpp"
#include "cyberdog_vision/body_detection.hpp"
#include "cyberdog_vision/face_recognition.hpp"
#include "cyberdog_vision/gesture_recognition.hpp"
#include "cyberdog_vision/keypoints_detection.hpp"
#include "cyberdog_vision/person_reid.hpp"
#include "cyberdog_vision/auto_track.hpp"
#include "cyberdog_vision/face_manager.hpp"

namespace cyberdog_vision
{

using BodyT = protocol::msg::Body;
using BodyInfoT = protocol::msg::BodyInfo;
using FaceT = protocol::msg::Face;
using FaceInfoT = protocol::msg::FaceInfo;
using KeypointT = protocol::msg::Keypoint;
using AlgoListT = protocol::msg::AlgoList;
using PersonInfoT = protocol::msg::Person;
using TrackResultT = protocol::msg::TrackResult;
using BodyRegionT = protocol::srv::BodyRegion;
using CameraServiceT = protocol::srv::CameraService;
using AlgoManagerT = protocol::srv::AlgoManager;
using FaceManagerT = protocol::srv::FaceManager;
using FaceResultT = protocol::msg::FaceResult;
using TrackingStatusT = protocol::msg::TrackingStatus;
using ReturnResultT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VisionManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  VisionManager();
  ~VisionManager();

protected:
  ReturnResultT on_configure(const rclcpp_lifecycle::State & state) override;
  ReturnResultT on_activate(const rclcpp_lifecycle::State & state) override;
  ReturnResultT on_deactivate(const rclcpp_lifecycle::State & state) override;
  ReturnResultT on_cleanup(const rclcpp_lifecycle::State & state) override;
  ReturnResultT on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  int Init();
  int InitIPC();
  void CreateObject();
  void CreateThread();
  void DestoryThread();
  void WakeThread(AlgoStruct & algo);
  void ResetThread(AlgoStruct & algo);
  void ResetAlgo();
  void ImageProc();
  void MainAlgoManager();
  void DependAlgoManager();
  void BodyDet();
  void FaceRecognize();
  void FocusTrack();
  void ReIDProc();
  void GestureRecognize();
  void KeypointsDet();
  void FaceDetProc(std::string);

  int LoadFaceLibrary(std::map<std::string, std::vector<float>> & library);
  int GetMatchBody(const sensor_msgs::msg::RegionOfInterest & roi);
  void SetAlgoState(const AlgoListT & algo_list, const bool & value);

  void publishFaceResult(
    int result, const std::string & name, cv::Mat & img,
    std::string & face_msg);

  void TrackingService(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<BodyRegionT::Request> req,
    std::shared_ptr<BodyRegionT::Response> res);

  void AlgoManagerService(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<AlgoManagerT::Request> req,
    std::shared_ptr<AlgoManagerT::Response> res);

  void FaceManagerService(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<FaceManagerT::Request> req,
    std::shared_ptr<FaceManagerT::Response> res);

  bool CallService(
    rclcpp::Client<CameraServiceT>::SharedPtr & client, const uint8_t & cmd,
    const std::string & args);

private:
  rclcpp::Service<BodyRegionT>::SharedPtr tracking_service_;
  rclcpp::Service<AlgoManagerT>::SharedPtr algo_manager_service_;
  rclcpp::Service<FaceManagerT>::SharedPtr facemanager_service_;
  rclcpp::Client<CameraServiceT>::SharedPtr camera_clinet_;

  rclcpp_lifecycle::LifecyclePublisher<PersonInfoT>::SharedPtr person_pub_;
  rclcpp_lifecycle::LifecyclePublisher<TrackingStatusT>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaceResultT>::SharedPtr face_result_pub_;

  std::shared_ptr<std::thread> img_proc_thread_;
  std::shared_ptr<std::thread> main_manager_thread_;
  std::shared_ptr<std::thread> depend_manager_thread_;
  std::shared_ptr<std::thread> body_det_thread_;
  std::shared_ptr<std::thread> face_thread_;
  std::shared_ptr<std::thread> focus_thread_;
  std::shared_ptr<std::thread> gesture_thread_;
  std::shared_ptr<std::thread> reid_thread_;
  std::shared_ptr<std::thread> keypoints_thread_;

  std::shared_ptr<BodyDetection> body_ptr_;
  std::shared_ptr<FaceRecognition> face_ptr_;
  std::shared_ptr<AutoTrack> focus_ptr_;
  std::shared_ptr<GestureRecognition> gesture_ptr_;
  std::shared_ptr<PersonReID> reid_ptr_;
  std::shared_ptr<KeypointsDetection> keypoints_ptr_;

  std::map<std::string, std::vector<float>> face_library_;

  GlobalImageBuf global_img_buf_;
  BodyResults body_results_;

  AlgoStruct face_struct_;
  AlgoStruct body_struct_;
  AlgoStruct gesture_struct_;
  AlgoStruct keypoints_struct_;
  AlgoStruct reid_struct_;
  AlgoStruct focus_struct_;
  AlgoProcess algo_proc_;

  std::mutex result_mtx_;
  PersonInfoT algo_result_;

  TrackingStatusT processing_status_;

  int shm_id_;
  int sem_set_id_;
  char * shm_addr_;

  size_t buf_size_;
  bool open_face_;
  bool open_body_;
  bool open_gesture_;
  bool open_keypoints_;
  bool open_reid_;
  bool open_focus_;
  bool is_activate_;
  bool face_detect_;

  bool main_algo_deactivated_;
  bool depend_deactivated_;
  bool body_deactivated_;
  bool face_deactivated_;
  bool focus_deactivated_;
  bool reid_deactivated_;
  bool gesture_deactivated_;
  bool keypoints_deactivated_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__VISION_MANAGER_HPP_
