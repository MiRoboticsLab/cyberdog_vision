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

#include <stdlib.h>

#include <utility>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <map>

#include "cyberdog_vision/vision_manager.hpp"
#include "cyberdog_vision/semaphore_op.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#define SHM_PROJ_ID 'A'
#define SEM_PROJ_ID 'B'

const int kKeypointsNum = 17;
const char kModelPath[] = "/SDCARD/vision";
const char kLibraryPath[] = "/home/mi/.faces/faceinfo.yaml";
namespace cyberdog_vision
{

VisionManager::VisionManager()
: rclcpp_lifecycle::LifecycleNode("vision_manager"), shm_addr_(nullptr), buf_size_(6),
  open_face_(false), open_body_(false), open_gesture_(false), open_keypoints_(false),
  open_reid_(false), open_focus_(false), is_activate_(false),
  main_algo_deactivated_(false), depend_deactivated_(false), body_deactivated_(false),
  face_deactivated_(false), focus_deactivated_(false), reid_deactivated_(false),
  gesture_deactivated_(false), keypoints_deactivated_(false), face_complated_(false),
  body_complated_(false), gesture_complated_(false), keypoints_complated_(false),
  reid_complated_(false), focus_complated_(false)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
}

ReturnResultT VisionManager::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Configuring vision_manager. ");
  if (0 != Init()) {
    return ReturnResultT::FAILURE;
  }
  INFO("Configure complated. ");
  return ReturnResultT::SUCCESS;
}

ReturnResultT VisionManager::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Activating vision_manager. ");
  if (!CallService(camera_clinet_, 0, "face-interval=1")) {
    ERROR("Start camera stream fail. ");
    return ReturnResultT::FAILURE;
  }
  INFO("Start camera stream success. ");
  is_activate_ = true;
  CreateThread();
  person_pub_->on_activate();
  status_pub_->on_activate();
  face_result_pub_->on_activate();
  processing_status_.status = TrackingStatusT::STATUS_SELECTING;
  INFO("Activate complated. ");
  return ReturnResultT::SUCCESS;
}

ReturnResultT VisionManager::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Deactivating vision_manager. ");
  is_activate_ = false;
  DestoryThread();
  ResetAlgo();
  INFO("Destory thread complated. ");
  if (!CallService(camera_clinet_, 0, "face-interval=0")) {
    ERROR("Close camera stream fail. ");
    return ReturnResultT::FAILURE;
  }
  INFO("Close camera stream success. ");
  person_pub_->on_deactivate();
  status_pub_->on_deactivate();
  face_result_pub_->on_deactivate();
  INFO("Deactivate success. ");
  return ReturnResultT::SUCCESS;
}

void ResetCudaDevs()
{
  int dev_count = 0;
  cudaSetDevice(dev_count);
  cudaDeviceReset();
  INFO("Cuda device reset complated. ");
}

ReturnResultT VisionManager::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Cleaning up vision_manager. ");
  img_proc_thread_.reset();
  main_manager_thread_.reset();
  depend_manager_thread_.reset();
  body_det_thread_.reset();
  face_thread_.reset();
  focus_thread_.reset();
  gesture_thread_.reset();
  reid_thread_.reset();
  keypoints_thread_.reset();
  person_pub_.reset();
  status_pub_.reset();
  face_result_pub_.reset();
  tracking_service_.reset();
  algo_manager_service_.reset();
  facemanager_service_.reset();
  camera_clinet_.reset();
  body_ptr_.reset();
  face_ptr_.reset();
  focus_ptr_.reset();
  gesture_ptr_.reset();
  reid_ptr_.reset();
  keypoints_ptr_.reset();
  ResetCudaDevs();
  INFO("Clean up complated. ");
  return ReturnResultT::SUCCESS;
}

ReturnResultT VisionManager::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Shutting down vision_manager. ");
  return ReturnResultT::SUCCESS;
}

int VisionManager::Init()
{
  if (0 != InitIPC()) {
    ERROR("Init shared memory or semaphore fail. ");
    return -1;
  }

  // Create object
  CreateObject();

  return 0;
}

int VisionManager::InitIPC()
{
  // Create shared memory and get address
  if (0 != CreateShm(SHM_PROJ_ID, sizeof(uint64_t) + IMAGE_SIZE, shm_id_)) {
    return -1;
  }
  shm_addr_ = GetShmAddr(shm_id_, sizeof(uint64_t) + IMAGE_SIZE);
  if (shm_addr_ == nullptr) {
    return -1;
  }

  // Create semaphore set, 0-mutex, 1-empty, 2-full
  if (0 != CreateSem(SEM_PROJ_ID, 3, sem_set_id_)) {
    return -1;
  }

  // Set init value of the semaphore
  if (0 != SetSemInitVal(sem_set_id_, 0, 1)) {
    return -1;
  }

  if (0 != SetSemInitVal(sem_set_id_, 1, 1)) {
    return -1;
  }

  if (0 != SetSemInitVal(sem_set_id_, 2, 0)) {
    return -1;
  }

  return 0;
}

void VisionManager::CreateObject()
{
  INFO("Create object start. ");
  // Create AI object
  body_ptr_ = std::make_shared<BodyDetection>(
    kModelPath + std::string("/body_gesture"));

  face_ptr_ = std::make_shared<FaceRecognition>(
    kModelPath + std::string(
      "/face_recognition"), true, true);

  focus_ptr_ = std::make_shared<AutoTrack>(
    kModelPath + std::string("/auto_track"));

  gesture_ptr_ = std::make_shared<GestureRecognition>(
    kModelPath + std::string("/body_gesture"));

  reid_ptr_ =
    std::make_shared<PersonReID>(
    kModelPath +
    std::string("/person_reid"));

  keypoints_ptr_ = std::make_shared<KeypointsDetection>(
    kModelPath + std::string("/keypoints_detection"));

  INFO("Create object complated. ");

  // Create service server
  tracking_service_ = create_service<BodyRegionT>(
    "tracking_object", std::bind(
      &VisionManager::TrackingService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  algo_manager_service_ = create_service<AlgoManagerT>(
    "algo_manager", std::bind(
      &VisionManager::AlgoManagerService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  facemanager_service_ = create_service<FaceManagerT>(
    "facemanager", std::bind(
      &VisionManager::FaceManagerService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create service client
  camera_clinet_ = create_client<CameraServiceT>("camera_service");

  // Create publisher
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  person_pub_ = create_publisher<PersonInfoT>("person", pub_qos);
  status_pub_ = create_publisher<TrackingStatusT>("processing_status", pub_qos);
  face_result_pub_ = create_publisher<FaceResultT>("facemanager/face_result", pub_qos);
}

void VisionManager::CreateThread()
{
  img_proc_thread_ = std::make_shared<std::thread>(&VisionManager::ImageProc, this);
  main_manager_thread_ = std::make_shared<std::thread>(&VisionManager::MainAlgoManager, this);
  depend_manager_thread_ = std::make_shared<std::thread>(&VisionManager::DependAlgoManager, this);
  body_det_thread_ = std::make_shared<std::thread>(&VisionManager::BodyDet, this);
  face_thread_ = std::make_shared<std::thread>(&VisionManager::FaceRecognize, this);
  focus_thread_ = std::make_shared<std::thread>(&VisionManager::FocusTrack, this);
  gesture_thread_ = std::make_shared<std::thread>(&VisionManager::GestureRecognize, this);
  reid_thread_ = std::make_shared<std::thread>(&VisionManager::ReIDProc, this);
  keypoints_thread_ = std::make_shared<std::thread>(&VisionManager::KeypointsDet, this);
}

void VisionManager::ImageProc()
{
  while (rclcpp::ok()) {
    if (!is_activate_) {
      INFO("ImageProc: Deactivate to return image thread. ");
      return;
    }
    if (0 != WaitSem(sem_set_id_, 2)) {return;}
    if (0 != WaitSem(sem_set_id_, 0)) {return;}
    StampedImage simg;
    simg.img.create(480, 640, CV_8UC3);
    memcpy(simg.img.data, reinterpret_cast<char *>(shm_addr_) + sizeof(uint64_t), IMAGE_SIZE);
    uint64_t time;
    memcpy(&time, reinterpret_cast<char *>(shm_addr_), sizeof(uint64_t));
    simg.header.stamp.sec = time / 1000000000;
    simg.header.stamp.nanosec = time % 1000000000;
    if (0 != SignalSem(sem_set_id_, 0)) {return;}
    if (0 != SignalSem(sem_set_id_, 1)) {return;}

    // Save image to buffer, only process with real img
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      global_img_buf_.img_buf.clear();
      global_img_buf_.img_buf.push_back(simg);
      global_img_buf_.is_filled = true;
      global_img_buf_.cond.notify_one();
      INFO("ImageProc: Notify main thread. ");
    }
  }
}

void VisionManager::MainAlgoManager()
{
  while (rclcpp::ok()) {
    {
      INFO("MainAlgoManager: Wait to activate main thread. ");
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      global_img_buf_.cond.wait(lk, [this] {return global_img_buf_.is_filled;});
      global_img_buf_.is_filled = false;
      INFO("MainAlgoManager: Activate main algo manager thread. ");
    }
    if (!is_activate_) {
      INFO("MainAlgoManager: Deactivate to return main algo thread. ");
      main_algo_deactivated_ = true;
      return;
    }

    if (open_body_) {
      std::unique_lock<std::mutex> lk_result(body_struct_.mtx);
      if (!body_struct_.is_called) {
        body_complated_ = false;
        reid_complated_ = false;
        gesture_complated_ = false;
        keypoints_complated_ = false;
        body_struct_.is_called = true;
        body_struct_.cond.notify_one();
      }
    }

    if (open_face_) {
      std::unique_lock<std::mutex> lk_result(face_struct_.mtx);
      if (!face_struct_.is_called) {
        face_complated_ = false;
        face_struct_.is_called = true;
        face_struct_.cond.notify_one();
      }
    }

    if (open_focus_) {
      std::unique_lock<std::mutex> lk_result(focus_struct_.mtx);
      if (!focus_struct_.is_called) {
        focus_complated_ = false;
        focus_struct_.is_called = true;
        focus_struct_.cond.notify_one();
      }
    }

    // Wait for result to pub
    if (open_body_ || open_face_ || open_focus_) {
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx);
      INFO("MainAlgoManager: main thread process_complated: %d", algo_proc_.process_complated);
      algo_proc_.cond.wait(lk_proc, [this] {return algo_proc_.process_complated;});
      INFO("MainAlgoManager: Main thread wake up to pub. ");
      {
        std::unique_lock<std::mutex> lk_result(result_mtx_);
        person_pub_->publish(algo_result_);
        INFO(
          "MainAlgoManager: Publish det stamp: %d.%d", algo_result_.body_info.header.stamp.sec,
          algo_result_.body_info.header.stamp.nanosec);
        INFO(
          "MainAlgoManager: Publish track stamp: %d.%d", algo_result_.track_res.header.stamp.sec,
          algo_result_.track_res.header.stamp.nanosec);
        for (size_t i = 0; i < algo_result_.body_info.infos.size(); ++i) {
          sensor_msgs::msg::RegionOfInterest rect = algo_result_.body_info.infos[i].roi;
          INFO(
            "MainAlgoManager: Publish detection %d bbox: %d,%d,%d,%d", i, rect.x_offset,
            rect.y_offset, rect.width, rect.height);
        }
        sensor_msgs::msg::RegionOfInterest rect = algo_result_.track_res.roi;
        INFO(
          "MainAlgoManager: Publish tracked bbox: %d,%d,%d,%d", rect.x_offset, rect.y_offset,
          rect.width,
          rect.height);
        PersonInfoT person_info;
        algo_result_ = person_info;
      }
      if (open_body_ || open_focus_) {
        status_pub_->publish(processing_status_);
      }
      algo_proc_.process_complated = false;
    }
    INFO("MainAlgoManager: end of main thread .");
  }
}

void VisionManager::DependAlgoManager()
{
  while (rclcpp::ok()) {
    {
      INFO("DependAlgoManager: Wait to activate depend thread. ");
      std::unique_lock<std::mutex> lk(body_results_.mtx);
      body_results_.cond.wait(lk, [this] {return body_results_.is_filled;});
      body_results_.is_filled = false;
      INFO("DependAlgoManager: Activate depend algo manager thread. ");
    }
    if (!is_activate_) {
      INFO("DependAlgoManager: Deactivate to return depend thread. ");
      depend_deactivated_ = true;
      return;
    }

    if (open_reid_) {
      std::unique_lock<std::mutex> lk_result(reid_struct_.mtx);
      if (!reid_struct_.is_called) {
        reid_struct_.is_called = true;
        reid_struct_.cond.notify_one();
      }
    }

    if (open_gesture_) {
      std::unique_lock<std::mutex> lk_result(gesture_struct_.mtx);
      if (!gesture_struct_.is_called) {
        gesture_struct_.is_called = true;
        gesture_struct_.cond.notify_one();
      }
    }

    if (open_keypoints_) {
      std::unique_lock<std::mutex> lk_result(keypoints_struct_.mtx);
      if (!keypoints_struct_.is_called) {
        keypoints_struct_.is_called = true;
        keypoints_struct_.cond.notify_one();
      }
    }
    INFO("DependAlgoManager: end of depend thread. ");
  }
}

cv::Rect Convert(const sensor_msgs::msg::RegionOfInterest & roi)
{
  cv::Rect bbox = cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
  return bbox;
}

sensor_msgs::msg::RegionOfInterest Convert(const cv::Rect & bbox)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = bbox.x;
  roi.y_offset = bbox.y;
  roi.width = bbox.width;
  roi.height = bbox.height;
  return roi;
}

void Convert(const std_msgs::msg::Header & header, const BodyFrameInfo & from, BodyInfoT & to)
{
  to.header = header;
  to.count = from.size();
  to.infos.clear();
  for (size_t i = 0; i < from.size(); ++i) {
    BodyT body;
    body.roi.x_offset = from[i].left;
    body.roi.y_offset = from[i].top;
    body.roi.width = from[i].width;
    body.roi.height = from[i].height;
    to.infos.push_back(body);
  }
}

void VisionManager::BodyDet()
{
  while (rclcpp::ok()) {
    {
      INFO("BodyDet: Wait to activate body thread. ");
      std::unique_lock<std::mutex> lk_struct(body_struct_.mtx);
      body_struct_.cond.wait(lk_struct, [this] {return body_struct_.is_called;});
      body_struct_.is_called = false;
      INFO("BodyDet: Activate body detect thread. ");
    }
    if (!is_activate_) {
      INFO("BodyDet: Deactivate to return body thread. ");
      body_deactivated_ = true;
      return;
    }

    // Get image and detect body
    StampedImage stamped_img;
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      stamped_img = global_img_buf_.img_buf.back();
    }

    BodyFrameInfo infos;
    {
      std::unique_lock<std::mutex> lk_body(body_results_.mtx);
      if (-1 != body_ptr_->Detect(stamped_img.img, infos)) {
        if (body_results_.body_infos.size() >= buf_size_) {
          body_results_.body_infos.erase(body_results_.body_infos.begin());
        }
        body_results_.body_infos.push_back(infos);
        body_results_.detection_img.img = stamped_img.img.clone();
        body_results_.detection_img.header = stamped_img.header;
        body_results_.is_filled = true;
        body_results_.cond.notify_one();
        INFO("BodyDet: Body thread notify depend thread. ");

        INFO("BodyDet: Body detection num: %d", infos.size());
        for (size_t count = 0; count < infos.size(); ++count) {
          INFO(
            "BodyDet: Person %d: sim: %f, x: %d", count, infos[count].score,
            infos[count].left);
        }
      } else {
        WARN("BodyDet: Body detect fail of current image. ");
      }
    }

    // Storage body detection result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      body_complated_ = true;
      algo_result_.header = stamped_img.header;
      Convert(stamped_img.header, infos, algo_result_.body_info);
      INFO(
        "BodyDet: Det result-img: %d.%d", stamped_img.header.stamp.sec,
        stamped_img.header.stamp.nanosec);
      INFO(
        "BodyDet: Det result-body: %d.%d", algo_result_.body_info.header.stamp.sec,
        algo_result_.body_info.header.stamp.nanosec);
      SetThreadState("BodyDet", algo_proc_.process_complated);
      INFO("BodyDet: body thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("BodyDet: Body thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void Convert(
  const std_msgs::msg::Header & header, const std::vector<MatchFaceInfo> & from,
  FaceInfoT & to)
{
  to.header = header;
  to.count = from.size();
  to.infos.clear();
  for (size_t i = 0; i < from.size(); ++i) {
    FaceT face;
    face.roi.x_offset = from[i].rect.left;
    face.roi.y_offset = from[i].rect.top;
    face.roi.width = from[i].rect.right - from[i].rect.left;
    face.roi.height = from[i].rect.bottom - from[i].rect.top;
    face.id = from[i].face_id;
    face.score = from[i].score;
    face.match = from[i].match_score;
    face.yaw = from[i].poses[0];
    face.pitch = from[i].poses[1];
    face.row = from[i].poses[2];
    face.age = from[i].ages[0];
    face.emotion = from[i].emotions[0];
    to.infos.push_back(face);
  }
}

void VisionManager::FaceRecognize()
{
  while (rclcpp::ok()) {
    {
      INFO("FaceRecognize: Wait to activate face thread. ");
      std::unique_lock<std::mutex> lk(face_struct_.mtx);
      face_struct_.cond.wait(lk, [this] {return face_struct_.is_called;});
      face_struct_.is_called = false;
      INFO("FaceRecognize: Activate face recognition thread. ");
    }
    if (!is_activate_) {
      INFO("FaceRecognize: Deactivate to return face thread. ");
      face_deactivated_ = true;
      return;
    }

    // Get image to proc
    StampedImage stamped_img;
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      stamped_img = global_img_buf_.img_buf.back();
    }

    // Face recognition and get result
    std::vector<MatchFaceInfo> result;
    if (0 != face_ptr_->GetRecognitionResult(stamped_img.img, face_library_, result)) {
      WARN("FaceRecognize: Face recognition fail. ");
    }

    // Storage face recognition result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      face_complated_ = true;
      algo_result_.header = stamped_img.header;
      Convert(stamped_img.header, result, algo_result_.face_info);
      SetThreadState("FaceRecognize", algo_proc_.process_complated);
      INFO("FaceRecognize: face thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("FaceRecognize: Face thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void Convert(
  const std_msgs::msg::Header & header, const cv::Rect & from, TrackResultT & to)
{
  to.header = header;
  to.roi = Convert(from);
}

void VisionManager::FocusTrack()
{
  while (rclcpp::ok()) {
    {
      INFO("FocusTrack: Wait to activate focus thread. ");
      std::unique_lock<std::mutex> lk(focus_struct_.mtx);
      focus_struct_.cond.wait(lk, [this] {return focus_struct_.is_called;});
      focus_struct_.is_called = false;
      INFO("FocusTrack: Activate focus thread. ");
    }
    if (!is_activate_) {
      INFO("FocusTrack: Deactivate to return focus thread. ");
      focus_deactivated_ = true;
      return;
    }

    // Get image to proc
    StampedImage stamped_img;
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      stamped_img = global_img_buf_.img_buf.back();
    }

    // Focus track and get result
    cv::Rect track_res = cv::Rect(0, 0, 0, 0);
    if (!focus_ptr_->Track(stamped_img.img, track_res)) {
      WARN("FocusTrack: Auto track fail of crunt frame. ");
    }
    if (focus_ptr_->GetLostStatus()) {
      WARN("FocusTrack: Auto track object lost. ");
      processing_status_.status = TrackingStatusT::STATUS_SELECTING;
    }

    // Storage foucs track result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      focus_complated_ = true;
      //  Convert data to publish
      INFO(
        "FocusTrack: Focus result %d,%d,%d,%d.", track_res.x, track_res.y, track_res.width,
        track_res.height);
      algo_result_.header = stamped_img.header;
      Convert(stamped_img.header, track_res, algo_result_.track_res);
      SetThreadState("FocusTrack", algo_proc_.process_complated);
      INFO("FocusTrack: focus thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("FocusTrack: Focus thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

double GetIOU(const HumanBodyInfo & b1, const sensor_msgs::msg::RegionOfInterest & b2)
{
  int w =
    std::max(
    std::min((b1.left + b1.width), (b2.x_offset + b2.width)) - std::max(
      b1.left,
      b2.x_offset),
    (uint32_t)0);
  int h =
    std::max(
    std::min((b1.top + b1.height), (b2.y_offset + b2.height)) - std::max(
      b1.top,
      b2.y_offset),
    (uint32_t)0);

  return w * h / static_cast<double>(b1.width * b1.height +
         b2.width * b2.height - w * h);
}

void VisionManager::ReIDProc()
{
  while (rclcpp::ok()) {
    int person_id = -1;
    cv::Rect tracked_bbox = cv::Rect(0, 0, 0, 0);
    {
      INFO("ReIDProc: Wait to activate reid thread. ");
      std::unique_lock<std::mutex> lk_reid(reid_struct_.mtx);
      reid_struct_.cond.wait(lk_reid, [this] {return reid_struct_.is_called;});
      reid_struct_.is_called = false;
      INFO("ReIDProc: Activate reid thread. ");
    }
    if (!is_activate_) {
      INFO("ReIDProc: Deactivate to return reid thread. ");
      reid_deactivated_ = true;
      return;
    }

    // ReID and get result
    cv::Mat img_show;
    std_msgs::msg::Header img_header;
    {
      INFO("ReIDProc: Waiting for mutex to reid. ");
      std::unique_lock<std::mutex> lk_body(body_results_.mtx);
      std::vector<InferBbox> body_bboxes = BodyConvert(body_results_.body_infos.back());
      img_show = body_results_.detection_img.img.clone();
      img_header = body_results_.detection_img.header;
      if (-1 !=
        reid_ptr_->GetReIDInfo(
          body_results_.detection_img.img, body_bboxes, person_id, tracked_bbox) &&
        -1 != person_id)
      {
        INFO(
          "ReIDProc: Reid result, person id: %d, bbox: %d, %d, %d, %d", person_id, tracked_bbox.x,
          tracked_bbox.y, tracked_bbox.width, tracked_bbox.height);
      }
      if (reid_ptr_->GetLostStatus()) {
        processing_status_.status = TrackingStatusT::STATUS_SELECTING;
      }
    }

    // Storage reid result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      reid_complated_ = true;
      Convert(img_header, tracked_bbox, algo_result_.track_res);
      SetThreadState("ReIDProc", algo_proc_.process_complated);
      INFO("ReIDProc: reid thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("ReIDProc: Reid thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void Convert(const std::vector<GestureInfo> & from, BodyInfoT & to)
{
  for (size_t i = 0; i < from.size(); ++i) {
    to.infos[i].gesture.roi = Convert(from[i].rect);
    to.infos[i].gesture.cls = from[i].label;
  }
}

void VisionManager::GestureRecognize()
{
  while (rclcpp::ok()) {
    {
      INFO("GestureRecognize: Wait to activate gesture thread. ");
      std::unique_lock<std::mutex> lk(gesture_struct_.mtx);
      gesture_struct_.cond.wait(lk, [this] {return gesture_struct_.is_called;});
      gesture_struct_.is_called = false;
      INFO("GestureRecognize: Activate gesture recognition thread. ");
    }
    if (!is_activate_) {
      INFO("GestureRecognize: Deactivate to return gesture thread. ");
      gesture_deactivated_ = true;
      return;
    }

    // Gesture recognition and get result
    bool is_success = false;
    std::vector<GestureInfo> infos;
    {
      std::unique_lock<std::mutex> lk_body(body_results_.mtx);
      std::vector<InferBbox> body_bboxes = BodyConvert(body_results_.body_infos.back());
      if (-1 != gesture_ptr_->GetGestureInfo(body_results_.detection_img.img, body_bboxes, infos)) {
        is_success = true;
      }
    }

    // Storage gesture recognition result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      gesture_complated_ = true;
      // Convert data to publish
      if (is_success) {
        Convert(infos, algo_result_.body_info);
      }
      SetThreadState("GestureRecognize", algo_proc_.process_complated);
      INFO("GestureRecognize: gesture thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("GestureRecognize: Gesture thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void Convert(const std::vector<std::vector<cv::Point2f>> & from, BodyInfoT & to)
{
  for (size_t i = 0; i < from.size(); ++i) {
    to.infos[i].keypoints.resize(kKeypointsNum * sizeof(cv::Point2f));
    for (size_t num = 0; num < from[i].size(); ++num) {
      KeypointT keypoint;
      keypoint.x = from[i][num].x;
      keypoint.y = from[i][num].y;
      to.infos[i].keypoints.push_back(keypoint);
    }
  }
}

void VisionManager::KeypointsDet()
{
  while (rclcpp::ok()) {
    {
      INFO("KeypointsDet: Wait to activate keypoints thread. ");
      std::unique_lock<std::mutex> lk(keypoints_struct_.mtx);
      keypoints_struct_.cond.wait(lk, [this] {return keypoints_struct_.is_called;});
      keypoints_struct_.is_called = false;
      INFO("KeypointsDet: Activate keypoints detection thread. ");
    }
    if (!is_activate_) {
      INFO("KeypointsDet: Deactivate to return keypoints thread. ");
      keypoints_deactivated_ = true;
      return;
    }

    // Keypoints detection and get result
    std::vector<std::vector<cv::Point2f>> bodies_keypoints;
    {
      std::unique_lock<std::mutex> lk_body(body_results_.mtx);
      std::vector<InferBbox> body_bboxes = BodyConvert(body_results_.body_infos.back());
      keypoints_ptr_->GetKeypointsInfo(
        body_results_.detection_img.img, body_bboxes,
        bodies_keypoints);
    }

    // Storage keypoints detection result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      keypoints_complated_ = true;
      // Convert data to publish
      Convert(bodies_keypoints, algo_result_.body_info);
      SetThreadState("KeypointsDet", algo_proc_.process_complated);
      INFO("KeypointsDet: keypoints thread process_complated: %d", algo_proc_.process_complated);
      if (algo_proc_.process_complated) {
        INFO("KeypointsDet: Keypoints thread notify to pub. ");
        algo_proc_.cond.notify_one();
      }
    }
  }
}

int VisionManager::LoadFaceLibrary(std::map<std::string, std::vector<float>> & library)
{
  cv::FileStorage fs(std::string(kLibraryPath), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ERROR("Open the face library file fail! ");
    return -1;
  }

  library.erase(library.begin(), library.end());
  cv::FileNode node = fs["UserFaceInfo"];
  cv::FileNodeIterator it = node.begin();
  for (; it != node.end(); ++it) {
    std::string name = (string)(*it)["name"];
    cv::FileNode feat = (*it)["feature"];
    cv::FileNodeIterator jt = feat.begin();
    std::vector<float> face_feat;
    for (; jt != feat.end(); ++jt) {
      face_feat.push_back(static_cast<float>(*jt));
    }
    library.insert(std::pair<std::string, std::vector<float>>(name, face_feat));
  }

  return 0;
}

int VisionManager::GetMatchBody(const sensor_msgs::msg::RegionOfInterest & roi)
{
  cv::Mat track_img;
  cv::Rect track_rect;
  bool is_found = false;
  for (size_t i = body_results_.body_infos.size() - 1; i > 0 && !is_found; --i) {
    double max_score = 0;
    for (size_t j = 0; j < body_results_.body_infos[i].size(); ++j) {
      double score = GetIOU(body_results_.body_infos[i][j], roi);
      if (score > max_score && score > 0.5) {
        max_score = score;
        track_rect = cv::Rect(
          body_results_.body_infos[i][j].left,
          body_results_.body_infos[i][j].top,
          body_results_.body_infos[i][j].width,
          body_results_.body_infos[i][j].height);
      }
    }
    if (max_score > 0.5) {
      is_found = true;
      track_img = body_results_.detection_img.img;
    }
  }
  if (is_found) {
    std::vector<float> reid_feat;
    if (0 != reid_ptr_->SetTracker(track_img, track_rect, reid_feat)) {
      WARN("Set reid tracker fail. ");
      return -1;
    }
  } else {
    WARN("Can not find match body. ");
    return -1;
  }
  return 0;
}

void VisionManager::SetAlgoState(const AlgoListT & algo_list, const bool & value)
{
  INFO("Algo type: %d", (int)algo_list.algo_module);
  switch (algo_list.algo_module) {
    case AlgoListT::ALGO_FACE:
      open_face_ = value;
      if (value) {
        LoadFaceLibrary(face_library_);
      }
      break;
    case AlgoListT::ALGO_BODY:
      open_body_ = value;
      break;
    case AlgoListT::ALGO_GESTURE:
      open_gesture_ = value;
      break;
    case AlgoListT::ALGO_KEYPOINTS:
      open_keypoints_ = value;
      break;
    case AlgoListT::ALGO_REID:
      open_reid_ = value;
      break;
    case AlgoListT::ALGO_FOCUS:
      open_focus_ = value;
      break;
    default:
      break;
  }
}

void VisionManager::TrackingService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<BodyRegionT::Request> req,
  std::shared_ptr<BodyRegionT::Response> res)
{
  INFO(
    "Received tracking object from app: %d, %d, %d, %d",
    req->roi.x_offset, req->roi.y_offset, req->roi.width, req->roi.height);

  if (open_reid_) {
    std::unique_lock<std::mutex> lk(body_results_.mtx);
    if (0 != GetMatchBody(req->roi)) {
      res->success = false;
    } else {
      res->success = true;
      processing_status_.status = TrackingStatusT::STATUS_TRACKING;
    }
  }

  if (open_focus_) {
    StampedImage stamped_img;
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      stamped_img = global_img_buf_.img_buf.back();
    }
    cv::Rect rect = Convert(req->roi);
    if (!focus_ptr_->SetTracker(stamped_img.img, rect)) {
      WARN("Set focus tracker fail. ");
      res->success = false;
    } else {
      res->success = true;
      processing_status_.status = TrackingStatusT::STATUS_TRACKING;
    }
  }
}

void VisionManager::AlgoManagerService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<AlgoManagerT::Request> req,
  std::shared_ptr<AlgoManagerT::Response> res)
{
  INFO("Received algo request. ");
  for (size_t i = 0; i < req->algo_enable.size(); ++i) {
    SetAlgoState(req->algo_enable[i], true);
  }
  for (size_t i = 0; i < req->algo_disable.size(); ++i) {
    SetAlgoState(req->algo_disable[i], false);
  }

  res->result_enable = AlgoManagerT::Response::ENABLE_SUCCESS;
  res->result_disable = AlgoManagerT::Response::DISABLE_SUCCESS;
}

bool VisionManager::CallService(
  rclcpp::Client<CameraServiceT>::SharedPtr & client,
  const uint8_t & cmd, const std::string & args)
{
  auto req = std::make_shared<CameraServiceT::Request>();
  req->command = cmd;
  req->args = args;

  std::chrono::seconds timeout = std::chrono::seconds(10);
  if (!client->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      ERROR("Interrupted while waiting for the service. Exiting.");
      return false;
    }
    INFO("Service not available...");
    return false;
  }

  auto client_cb = [timeout](rclcpp::Client<CameraServiceT>::SharedFuture future) {
      std::future_status status = future.wait_for(timeout);

      if (status == std::future_status::ready) {
        if (0 != future.get()->result) {
          return false;
        } else {
          return true;
        }
      } else {
        return false;
      }
    };

  auto result = client->async_send_request(req, client_cb);
  return true;
}

void VisionManager::FaceManagerService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<FaceManagerT::Request> request,
  std::shared_ptr<FaceManagerT::Response> response)
{
  INFO(
    "face service received command %d, argument '%s'",
    request->command, request->args.c_str());

  switch (request->command) {
    case FaceManagerT::Request::ADD_FACE:
      cout << "addFaceInfo: " << request->username << " is_host: " << request->ishost << endl;
      if (request->username.length() == 0) {
        response->result = -1;
      } else {
        FaceManager::getInstance()->addFaceIDCacheInfo(request->username, request->ishost);
        face_detect_ = true;
        std::thread faceDet = std::thread(&VisionManager::FaceDetProc, this, request->username);
        faceDet.detach();
        response->result = 0;
      }
      break;
    case FaceManagerT::Request::CANCLE_ADD_FACE:
      cout << "cancelAddFace" << endl;
      face_detect_ = false;
      response->result = FaceManager::getInstance()->cancelAddFace();
      break;
    case FaceManagerT::Request::CONFIRM_LAST_FACE:
      cout << "confirmFace username:" << request->username << " is_host:" << request->ishost <<
        endl;
      if (request->username.length() == 0) {
        response->result = -1;
      } else {
        response->result = FaceManager::getInstance()->confirmFace(
          request->username,
          request->ishost);
      }
      break;
    case FaceManagerT::Request::UPDATE_FACE_ID:
      cout << "updateFaceId username:" << request->username << " ori_name:" << request->oriname <<
        endl;
      if (request->username.length() == 0 || request->oriname.length() == 0) {
        response->result = -1;
      } else {
        response->result = FaceManager::getInstance()->updateFaceId(
          request->oriname,
          request->username);
      }
      break;
    case FaceManagerT::Request::DELETE_FACE:
      cout << "deleteFace username:" << request->username << endl;
      if (request->username.length() == 0) {
        response->result = -1;
      } else {
        response->result = FaceManager::getInstance()->deleteFace(request->username);
      }
      break;
    case FaceManagerT::Request::GET_ALL_FACES:
      response->msg = FaceManager::getInstance()->getAllFaces();
      cout << "getAllFaces " << response->msg << endl;
      response->result = 0;
      break;
    default:
      ERROR("service unsupport command %d", request->command);
      response->result = FaceManagerT::Response::RESULT_INVALID_ARGS;
  }
}

void VisionManager::publishFaceResult(
  int result, const std::string & face_name, cv::Mat & img,
  std::string & face_msg)
{
  auto face_result_msg = std::make_unique<FaceResultT>();
  size_t png_size;
  unsigned char * png_data;

  face_result_msg->result = result;
  face_result_msg->msg = face_msg;

  if (result == 0 || result == 17) {
    std::vector<unsigned char> png_buff;
    std::vector<int> png_param = std::vector<int>(2);
    png_param[0] = 16;  // CV_IMWRITE_PNG_QUALITY;
    png_param[1] = 3;  // default(95)

    imencode(".png", img, png_buff, png_param);
    png_size = png_buff.size();
    png_data = (unsigned char *)malloc(png_size);
    for (size_t i = 0; i < png_size; i++) {
      png_data[i] = png_buff[i];
    }
    face_result_msg->face_images.resize(1);
    face_result_msg->face_images[0].header.frame_id = face_name;
    face_result_msg->face_images[0].format = "png";
    face_result_msg->face_images[0].data.resize(png_size);
    memcpy(&(face_result_msg->face_images[0].data[0]), png_data, png_size);
    face_result_pub_->publish(std::move(face_result_msg));

    free(png_data);
  } else {
    face_result_pub_->publish(std::move(face_result_msg));
  }
}

void VisionManager::FaceDetProc(std::string face_name)
{
  std::map<std::string, std::vector<float>> endlib_feats;
  std::vector<MatchFaceInfo> match_info;
  cv::Mat mat_tmp;
  bool get_face_timeout = true;
  std::string checkFacePose_Msg;
  int checkFacePose_ret;
  endlib_feats = FaceManager::getInstance()->getFeatures();
  std::time_t cur_time = std::time(NULL);

  while (std::difftime(std::time(NULL), cur_time) < 40 && face_detect_) {
    get_face_timeout = false;
    std::unique_lock<std::mutex> lk_img(global_img_buf_.mtx);
    global_img_buf_.cond.wait(lk_img, [this] {return global_img_buf_.is_filled;});
    global_img_buf_.is_filled = false;

    std::vector<EntryFaceInfo> faces_info;
    mat_tmp = global_img_buf_.img_buf[0].img.clone();

    face_ptr_->GetFaceInfo(mat_tmp, faces_info);
#if 0
    // debug - visualization
    for (unsigned int i = 0; i < faces_info.size(); i++) {
      cv::rectangle(
        mat_tmp,
        cv::Rect(
          faces_info[i].rect.left, faces_info[i].rect.top,
          (faces_info[i].rect.right - faces_info[i].rect.left),
          (faces_info[i].rect.bottom - faces_info[i].rect.top)),
        cv::Scalar(0, 0, 255));
    }
    cv::imshow("face", mat_tmp);
    cv::waitKey(10);
#endif
    checkFacePose_ret = FaceManager::getInstance()->checkFacePose(faces_info, checkFacePose_Msg);
    if (checkFacePose_ret == 0) {
      /*check if face feature already in endlib_feats*/
      face_ptr_->GetRecognitionResult(mat_tmp, endlib_feats, match_info);
      if (match_info.size() > 0 && match_info[0].match_score > 0.65) {
        checkFacePose_ret = 17;
        face_name = match_info[0].face_id;
        checkFacePose_Msg = "face already in endlib";
        ERROR(
          "%s face already in endlib current score:%f",
          face_name.c_str(), match_info[0].match_score);
      } else {
        FaceManager::getInstance()->addFaceFeatureCacheInfo(faces_info);
      }
    }
    publishFaceResult(checkFacePose_ret, face_name, mat_tmp, checkFacePose_Msg);
    if (checkFacePose_ret == 0 || checkFacePose_ret == 17) {
      break;
    }
    get_face_timeout = true;
  }

  /*it time out publish error*/
  if (get_face_timeout && face_detect_) {
    checkFacePose_Msg = "timeout";
    publishFaceResult(3, face_name, mat_tmp, checkFacePose_Msg);
  }
}

void Download(const std::string & name)
{
  cyberdog::common::cyberdog_model test(name);
  test.SetTimeout(300);
  int32_t code = test.UpdateModels();
  if (0 == code) {
    INFO("Update model from Fds success. ");
  } else {
    ERROR("Update model %s from Fds fail.", name.c_str());
  }
}

void VisionManager::SetThreadState(const std::string & thread_flag, bool & state)
{
  INFO("%s: Face: %d", thread_flag.c_str(), !open_face_ || face_complated_);
  INFO("%s: Body: %d", thread_flag.c_str(), !open_body_ || body_complated_);
  INFO("%s: Focus: %d", thread_flag.c_str(), !open_focus_ || focus_complated_);
  INFO("%s: Keypoints: %d", thread_flag.c_str(), !open_keypoints_ || keypoints_complated_);
  INFO("%s: Gesture: %d", thread_flag.c_str(), !open_gesture_ || gesture_complated_);
  INFO("%s: ReID: %d", thread_flag.c_str(), !open_reid_ || reid_complated_);
  state = (!open_face_ || face_complated_) && (!open_body_ || body_complated_) &&
    (!open_gesture_ || gesture_complated_) && (!open_keypoints_ || keypoints_complated_) &&
    (!open_reid_ || reid_complated_) && (!open_focus_ || focus_complated_);
}

void VisionManager::WakeThread(AlgoStruct & algo)
{
  std::unique_lock<std::mutex> lk(algo.mtx);
  if (!algo.is_called) {
    algo.is_called = true;
    algo.cond.notify_one();
  }
}

void VisionManager::ResetThread(AlgoStruct & algo)
{
  std::unique_lock<std::mutex> lk(algo.mtx);
  algo.is_called = false;
}

void VisionManager::ResetAlgo()
{
  focus_ptr_->ResetTracker();
  reid_ptr_->ResetTracker();
  open_face_ = false;
  open_body_ = false;
  open_gesture_ = false;
  open_keypoints_ = false;
  open_reid_ = false;
  open_focus_ = false;
  main_algo_deactivated_ = false;
  depend_deactivated_ = false;
  body_deactivated_ = false;
  face_deactivated_ = false;
  focus_deactivated_ = false;
  reid_deactivated_ = false;
  gesture_deactivated_ = false;
  keypoints_deactivated_ = false;
  face_complated_ = false;
  body_complated_ = false;
  gesture_complated_ = false;
  keypoints_complated_ = false;
  reid_complated_ = false;
  focus_complated_ = false;
  ResetThread(body_struct_);
  ResetThread(face_struct_);
  ResetThread(focus_struct_);
  ResetThread(reid_struct_);
  ResetThread(gesture_struct_);
  ResetThread(keypoints_struct_);
  {
    std::unique_lock<std::mutex> lk_body(body_results_.mtx);
    body_results_.is_filled = false;
  }
  {
    std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
    global_img_buf_.is_filled = false;
  }
  {
    std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx);
    algo_proc_.process_complated = false;
  }
}

void VisionManager::DestoryThread()
{
  if (img_proc_thread_->joinable()) {
    img_proc_thread_->join();
    INFO("img_proc_thread_ joined. ");
  }

  if (gesture_thread_->joinable()) {
    if (!gesture_deactivated_) {
      WakeThread(gesture_struct_);
    }
    gesture_thread_->join();
    INFO("gesture_thread_ joined. ");
  }

  if (reid_thread_->joinable()) {
    if (!reid_deactivated_) {
      WakeThread(reid_struct_);
    }
    reid_thread_->join();
    INFO("reid_thread_ joined. ");
  }

  if (keypoints_thread_->joinable()) {
    if (!keypoints_deactivated_) {
      WakeThread(keypoints_struct_);
    }
    keypoints_thread_->join();
    INFO("keypoints_thread_ joined. ");
  }

  if (face_thread_->joinable()) {
    if (!face_deactivated_) {
      WakeThread(face_struct_);
    }
    face_thread_->join();
    INFO("face_thread_ joined. ");
  }

  if (focus_thread_->joinable()) {
    if (!focus_deactivated_) {
      WakeThread(focus_struct_);
    }
    focus_thread_->join();
    INFO("focus_thread_ joined. ");
  }

  if (body_det_thread_->joinable()) {
    if (!body_deactivated_) {
      WakeThread(body_struct_);
    }
    body_det_thread_->join();
    INFO("body_det_thread_ joined. ");
  }

  {
    std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx);
    algo_proc_.process_complated = true;
    algo_proc_.cond.notify_one();
    INFO("Destory notify to pub. ");
  }
  if (depend_manager_thread_->joinable()) {
    if (!depend_deactivated_) {
      std::unique_lock<std::mutex> lk_body(body_results_.mtx);
      if (!body_results_.is_filled) {
        body_results_.is_filled = true;
        body_results_.cond.notify_one();
        INFO("Destory notify depend thread. ");
      }
    }
    depend_manager_thread_->join();
    INFO("depend_manager_thread_ joined. ");
  }

  if (main_manager_thread_->joinable()) {
    if (!main_algo_deactivated_) {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      if (!global_img_buf_.is_filled) {
        global_img_buf_.is_filled = true;
        global_img_buf_.cond.notify_one();
      }
    }
    main_manager_thread_->join();
    INFO("main_manager_thread_ joined. ");
  }
}

VisionManager::~VisionManager()
{
  DestoryThread();

  if (0 == DetachShm(shm_addr_)) {
    DelShm(shm_id_);
  }
}

}  // namespace cyberdog_vision
