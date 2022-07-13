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

#include "cyberdog_vision/vision_manager.hpp"
#include "cyberdog_vision/semaphore_op.hpp"

#define SHM_PROJ_ID 'A'
#define SEM_PROJ_ID 'B'

const std::string kModelPath = "/SDCARD/ros2_ws/src/cyberdog_vision/3rdparty";
const std::string kLibraryPath = "/home/mi/.faces/faceinfo.yaml";
namespace cyberdog_vision
{

VisionManager::VisionManager()
: Node("vision_manager"), shm_addr_(nullptr), buf_size_(6), is_tracking_(false),
  open_face_(false), open_body_(false), open_gesture_(false), open_keypoints_(false),
  open_reid_(false), open_focus_(false)
{
  if (0 != Init()) {
    throw std::logic_error("Init shared memory or semaphore fail. ");
  }

  // Create object
  CreateObject();
  if (!CallService(camera_clinet_, 0, "face-interval=1")) {
    throw std::logic_error("Start camera stream fail. ");
  }

  // Create process thread
  img_proc_thread_ = std::make_shared<std::thread>(&VisionManager::ImageProc, this);
  main_manager_thread_ = std::make_shared<std::thread>(&VisionManager::MainAlgoManager, this);
  depend_manager_thread_ = std::make_shared<std::thread>(&VisionManager::DependAlgoManager, this);
  body_det_thread_ = std::make_shared<std::thread>(&VisionManager::BodyDet, this);
  face_thread_ = std::make_shared<std::thread>(&VisionManager::FaceRecognize, this);
  focus_thread_ = std::make_shared<std::thread>(&VisionManager::FocusTrack, this);
  reid_thread_ = std::make_shared<std::thread>(&VisionManager::ReIDProc, this);
  gesture_thread_ = std::make_shared<std::thread>(&VisionManager::GestureRecognize, this);
  keypoints_thread_ = std::make_shared<std::thread>(&VisionManager::KeypointsDet, this);

}

int VisionManager::Init()
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
  // Create AI object
  body_ptr_ = std::make_shared<BodyDetection>(
    kModelPath + "/body_gesture/model/detect.onnx",
    kModelPath + "/body_gesture/model/cls_human_mid.onnx");

  reid_ptr_ = std::make_shared<PersonReID>(kModelPath + "/person_reid/model/reid_v1_mid.engine");
  face_ptr_ = std::make_shared<FaceRecognition>(kModelPath + "/face_recognition", true, true);

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

  face_result_pub_ = create_publisher<FaceResultT>("/facemanager/face_result", pub_qos);
}

void VisionManager::ImageProc()
{
  while (rclcpp::ok()) {
    WaitSem(sem_set_id_, 2);
    WaitSem(sem_set_id_, 0);
    StampedImage simg;
    simg.img.create(480, 640, CV_8UC3);
    memcpy(simg.img.data, (char *)shm_addr_ + sizeof(uint64_t), IMAGE_SIZE);
    uint64_t time;
    memcpy(&time, (char *)shm_addr_, sizeof(uint64_t));
    simg.header.stamp.sec = time / 1000000000;
    simg.header.stamp.nanosec = time % 1000000000;
    SignalSem(sem_set_id_, 0);
    SignalSem(sem_set_id_, 1);

    // Save image to buffer, only process with real img
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      global_img_buf_.img_buf.clear();
      global_img_buf_.img_buf.push_back(simg);
      global_img_buf_.is_filled = true;
      global_img_buf_.cond.notify_one();
    }
  }
}

void VisionManager::MainAlgoManager()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
      global_img_buf_.cond.wait(lk, [this] {return global_img_buf_.is_filled;});
      global_img_buf_.is_filled = false;
      std::cout << "===Activate main algo manager thread. " << std::endl;
    }

    if (open_body_) {
      std::lock(algo_proc_.mtx, body_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(body_struct_.mtx, std::adopt_lock);
      if (!body_struct_.is_called) {
        algo_proc_.process_num++;
        body_struct_.is_called = true;
        body_struct_.cond.notify_one();
      }
    }

    if (open_face_) {
      std::lock(algo_proc_.mtx, face_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(face_struct_.mtx, std::adopt_lock);
      if (!face_struct_.is_called) {
        algo_proc_.process_num++;
        face_struct_.is_called = true;
        face_struct_.cond.notify_one();
      }
    }

    if (open_focus_) {
      std::lock(algo_proc_.mtx, focus_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(focus_struct_.mtx, std::adopt_lock);
      if (!focus_struct_.is_called) {
        algo_proc_.process_num++;
        focus_struct_.is_called = true;
        focus_struct_.cond.notify_one();
      }
    }

    // Wait for result to pub
    if (!open_gesture_ && !open_keypoints_ && !open_reid_) {
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx);
      algo_proc_.cond.wait(lk_proc, [this] {return 0 == algo_proc_.process_num;});
      {
        std::unique_lock<std::mutex> lk_result(result_mtx_);
        person_pub_->publish(algo_result_);
      }
    }
  }
}

void VisionManager::DependAlgoManager()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(body_results_.mtx);
      body_results_.cond.wait(lk, [this] {return body_results_.is_filled;});
      body_results_.is_filled = false;
      std::cout << "===Activate depend algo manager thread. " << std::endl;
    }

    if (open_reid_) {
      std::lock(algo_proc_.mtx, reid_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(reid_struct_.mtx, std::adopt_lock);
      if (!reid_struct_.is_called) {
        algo_proc_.process_num++;
        reid_struct_.is_called = true;
        reid_struct_.cond.notify_one();
      }
    }

    if (open_gesture_) {
      std::lock(algo_proc_.mtx, gesture_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(gesture_struct_.mtx, std::adopt_lock);
      if (!gesture_struct_.is_called) {
        algo_proc_.process_num++;
        gesture_struct_.is_called = true;
        gesture_struct_.cond.notify_one();
      }
    }

    if (open_keypoints_) {
      std::lock(algo_proc_.mtx, keypoints_struct_.mtx);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(keypoints_struct_.mtx, std::adopt_lock);
      if (!keypoints_struct_.is_called) {
        algo_proc_.process_num++;
        keypoints_struct_.is_called = true;
        keypoints_struct_.cond.notify_one();
      }
    }

    // Wait for result to pub
    if (open_gesture_ || open_keypoints_ || open_reid_) {
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx);
      algo_proc_.cond.wait(lk_proc, [this] {return 0 == algo_proc_.process_num;});
      {
        std::unique_lock<std::mutex> lk_result(result_mtx_);
        person_pub_->publish(algo_result_);
      }
    }
  }
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
      std::unique_lock<std::mutex> lk_struct(body_struct_.mtx);
      body_struct_.cond.wait(lk_struct, [this] {return body_struct_.is_called;});
      body_struct_.is_called = false;
      std::cout << "===Activate body detect thread. " << std::endl;
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

        // debug - visualization
        // std::cout << "Detection result: " << std::endl;
        // std::cout << "Person num: " << infos.size() << std::endl;
        // cv::Mat img_show = stamped_img.img.clone();
        // for (auto & res : infos) {
        //   cv::rectangle(
        //     img_show, cv::Rect(res.left, res.top, res.width, res.height),
        //     cv::Scalar(0, 0, 255));
        // }
        // cv::imshow("vision", img_show);
        // cv::waitKey(10);
      } else {
        RCLCPP_WARN(get_logger(), "Body detect fail of current image. ");
      }
    }

    // Storage body detection result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;
      Convert(stamped_img.header, infos, algo_result_.body_info);
      if (0 == algo_proc_.process_num) {
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
    face.score = from[i].score;
    face.match = from[i].match_score;
    face.yaw = from[i].poses[0];
    face.pitch = from[i].poses[1];
    face.row = from[i].poses[2];
    to.infos.push_back(face);
  }
}

void VisionManager::FaceRecognize()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(face_struct_.mtx);
      face_struct_.cond.wait(lk, [this] {return face_struct_.is_called;});
      face_struct_.is_called = false;
      std::cout << "===Activate face recognition thread. " << std::endl;
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
      RCLCPP_WARN(this->get_logger(), "Face recognition fail. ");
    }

    // Storage face recognition result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;
      Convert(stamped_img.header, result, algo_result_.face_info);
      if (0 == algo_proc_.process_num) {
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void VisionManager::FocusTrack()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(focus_struct_.mtx);
      focus_struct_.cond.wait(lk, [this] {return focus_struct_.is_called;});
      focus_struct_.is_called = false;
      std::cout << "===Activate focus thread. " << std::endl;
    }

    // TODO: focus track and get result

    // Storage foucs track result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;

      // TODO: Convert data to publish

      if (0 == algo_proc_.process_num) {
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void VisionManager::ReIDProc()
{
  while (rclcpp::ok()) {
    int person_id = -1;
    size_t person_index;
    {
      std::unique_lock<std::mutex> lk_reid(reid_struct_.mtx);
      reid_struct_.cond.wait(lk_reid, [this] {return reid_struct_.is_called;});
      reid_struct_.is_called = false;
      {
        std::unique_lock<std::mutex> lk_body(body_results_.mtx, std::adopt_lock);
        std::vector<InferBbox> body_bboxes = BodyConvert(body_results_.body_infos.back());
        if (-1 !=
          reid_ptr_->GetReIDInfo(
            body_results_.detection_img.img, body_bboxes, person_id,
            person_index) &&
          -1 != person_id)
        {
          RCLCPP_INFO(get_logger(), "Reid result, person id: %d", person_id);
        }
      }
    }

    // Storage reid result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;
      if (-1 != person_id) {
        algo_result_.body_info.infos[person_index].reid = std::to_string(person_id);
      }
      if (0 == algo_proc_.process_num) {
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void VisionManager::GestureRecognize()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(gesture_struct_.mtx);
      gesture_struct_.cond.wait(lk, [this] {return gesture_struct_.is_called;});
      gesture_struct_.is_called = false;
      std::cout << "===Activate gesture recognition thread. " << std::endl;
    }

    // TODO: gesture recognition and get result

    // Storage gesture recognition result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;
      // TODO: Convert data to publish

      if (0 == algo_proc_.process_num) {
        algo_proc_.cond.notify_one();
      }
    }
  }
}

void VisionManager::KeypointsDet()
{
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lk(keypoints_struct_.mtx);
      keypoints_struct_.cond.wait(lk, [this] {return keypoints_struct_.is_called;});
      keypoints_struct_.is_called = false;
      std::cout << "===Activate keypoints detection thread. " << std::endl;
    }

    // TODO: keypoints detection and get result

    // Storage keypoints detection result
    {
      std::lock(algo_proc_.mtx, result_mtx_);
      std::unique_lock<std::mutex> lk_proc(algo_proc_.mtx, std::adopt_lock);
      std::unique_lock<std::mutex> lk_result(result_mtx_, std::adopt_lock);
      algo_proc_.process_num--;
      // TODO: Convert data to publish

      if (0 == algo_proc_.process_num) {
        algo_proc_.cond.notify_one();
      }
    }
  }
}

int VisionManager::LoadFaceLibrary(std::map<std::string, std::vector<float>> & library)
{
  cv::FileStorage fs(kLibraryPath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Open the face library file fail! ");
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
      face_feat.push_back((float)(*jt));
    }
    library.insert(std::pair<std::string, std::vector<float>>(name, face_feat));
  }

  return 0;
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
    is_tracking_ = true;
    std::vector<float> reid_feat;
    if (0 != reid_ptr_->SetTracker(track_img, track_rect, reid_feat)) {
      RCLCPP_WARN(get_logger(), "Set tracker fail. ");
      return -1;
    }
  } else {
    return -1;
  }
  return 0;
}

void VisionManager::SetAlgoState(const AlgoListT & algo_list, const bool & value)
{
  std::cout << "Algo type: " << algo_list.algo_module << std::endl;
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
  RCLCPP_INFO(
    this->get_logger(), "Received tracking object from app: %d, %d, %d, %d",
    req->roi.x_offset, req->roi.y_offset, req->roi.width, req->roi.height);
  std::unique_lock<std::mutex> lk(body_results_.mtx);
  if (0 != GetMatchBody(req->roi)) {
    res->success = false;
  } else {
    res->success = true;
  }
}

void VisionManager::AlgoManagerService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<AlgoManagerT::Request> req,
  std::shared_ptr<AlgoManagerT::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Received algo request. ");
  std::cout << "Algo enable: " << std::endl;
  for (size_t i = 0; i < req->algo_enable.size(); ++i) {
    SetAlgoState(req->algo_enable[i], true);
  }
  for (size_t i = 0; i < req->algo_disable.size(); ++i) {
    SetAlgoState(req->algo_disable[i], false);
  }

  res->result_enable = AlgoManagerT::Response::ENABLE_SUCCESS;
  res->result_disable = AlgoManagerT::Response::DISABLE_SUCCESS;
}

void VisionManager::FaceManagerService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<FaceManagerT::Request> request,
  std::shared_ptr<FaceManagerT::Response> response)
{
  RCLCPP_ERROR(
    this->get_logger(), "face service received command %d, argument '%s'",
    request->command, request->args.c_str());

  switch (request->command) {
    case FaceManagerT::Request::ADD_FACE:
      response->result = addFaceInfo(request->args);
      break;
    case FaceManagerT::Request::CANCLE_ADD_FACE:
      response->result = cancelAddFace();
      break;
    case FaceManagerT::Request::CONFIRM_LAST_FACE:
      response->result = confirmFace(request->args);
      break;
    case FaceManagerT::Request::UPDATE_FACE_ID:
      response->result = updateFaceId(request->args);
      break;
    case FaceManagerT::Request::DELETE_FACE:
      response->result = deleteFace(request->args);
      break;
    case FaceManagerT::Request::GET_ALL_FACES:
      response->result = getAllFaces(response);
      //response->result = startFaceMatch();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "service unsupport command %d", request->command);
      response->result = FaceManagerT::Response::RESULT_INVALID_ARGS;
  }
}


void split_string(
  const std::string & s,
  std::vector<std::string> & v, const std::string & sp)
{
  std::string::size_type pos1, pos2;
  pos2 = s.find(sp);
  pos1 = 0;

  while (std::string::npos != pos2) {
    v.push_back(s.substr(pos1, pos2));
    pos1 = pos2 + sp.size();
    pos2 = s.find(sp, pos1);
  }
  if (pos1 != s.length()) {
    v.push_back(s.substr(pos1));
  }
}


std::map<std::string, std::string> parse_parameters(std::string & params)
{
  std::vector<std::string> key_values;
  std::map<std::string, std::string> params_map;

  split_string(params, key_values, ";");
  for (size_t i = 0; i < key_values.size(); i++) {
    size_t pos = key_values[i].find("=");
    if (std::string::npos != pos) {
      std::string key = key_values[i].substr(0, pos);
      std::string value = key_values[i].substr(pos + 1);
      params_map[key] = value;
    }
  }

  return params_map;
}

void VisionManager::publishFaceResult(int result, const std::string & name, cv::Mat & img)
{
  auto face_result_msg = std::make_unique<FaceResultT>();
  printf("publishFaceResult:%d %s\n", result, name.c_str());
  size_t png_size;
  unsigned char * png_data;

  std::vector<unsigned char> png_buff;
  std::vector<int> png_param = std::vector<int>(2);
  png_param[0] = 16; //CV_IMWRITE_PNG_QUALITY;
  png_param[1] = 3;  //default(95)

  imencode(".png", img, png_buff, png_param);
  png_size = png_buff.size();
  png_data = (unsigned char *)malloc(png_size);
  for (size_t i = 0; i < png_size; i++) {
    png_data[i] = png_buff[i];
  }

#if 0
  /*save to png file*/
  std::string filename;
  FILE * fp;
  filename = "/home/mi/.faces/test.png";
  fp = fopen(filename.c_str(), "wb+");
  size_t remain = size;
  int writeptr = 0;
  int count = 0;
  while (remain > 0) {
    count = fwrite(mat_data + writeptr, 1, remain, fp);
    writeptr += count;
    remain -= count;
  }
  fclose(fp);
#endif

  face_result_msg->result = result;
  face_result_msg->msg = name;
  face_result_msg->face_images.resize(1);
  face_result_msg->face_images[0].header.frame_id = name;
  face_result_msg->face_images[0].format = "png";
  face_result_msg->face_images[0].data.resize(png_size);
  //std::copy(buff.begin(),buff.end(),&(msg->face_images[0].data[0]));
  memcpy(&(face_result_msg->face_images[0].data[0]), png_data, png_size);
  face_result_pub_->publish(std::move(face_result_msg));

  free(png_data);
}


void VisionManager::FaceDetProc(std::string face_name)
{
  std::map<std::string, std::vector<float>> endlib_feats;
  std::vector<MatchFaceInfo> match_info;
  cv::Mat mat_tmp;
  bool get_face_success = false;
  endlib_feats = FaceManager::getInstance()->getFeatures();
  std::time_t cur_time = std::time(NULL);
  while (std::difftime(std::time(NULL), cur_time) < 10) {
    std::unique_lock<std::mutex> lk_img(global_img_buf_.mtx, std::adopt_lock);
    global_img_buf_.cond.wait(lk_img, [this] {return global_img_buf_.is_filled;});
    global_img_buf_.is_filled = false;

    std::vector<EntryFaceInfo> faces_info;
    mat_tmp = global_img_buf_.img_buf[0].img.clone();

    face_ptr_->GetFaceInfo(mat_tmp, faces_info);

    if (FaceManager::getInstance()->checkFacePose(faces_info)) {
      /*check if face feature already in endlib_feats*/
      face_ptr_->GetRecognitionResult(mat_tmp, endlib_feats, match_info);
      //printf("match_info:match_score:%f\n",match_info[0].match_score);
      if (match_info.size() > 0 && match_info[0].match_score > 0.9) {
        publishFaceResult(-1, match_info[0].face_id, mat_tmp);
        RCLCPP_ERROR(this->get_logger(), "%s already in endlib\n", match_info[0].face_id.c_str());
        get_face_success = true;
        break;
      }
      /*get face info sucess*/
      FaceManager::getInstance()->addFaceFeatureCacheInfo(faces_info);
      publishFaceResult(0, face_name, mat_tmp);
      get_face_success = true;
      break;
    }

  }
  /*it time out publish error*/
  if (!get_face_success) {
    publishFaceResult(-1, "timeout", mat_tmp);
  }
}


int VisionManager::startFaceMatch()
{
  std::vector<MatchFaceInfo> match_info;
  std::map<std::string, std::vector<float>> endlib_feats;

  endlib_feats = FaceManager::getInstance()->getFeatures();

  while (1) {
    std::unique_lock<std::mutex> lk_img(global_img_buf_.mtx, std::adopt_lock);
    global_img_buf_.cond.wait(lk_img, [this] {return global_img_buf_.is_filled;});
    global_img_buf_.is_filled = false;

    cv::Mat mat_tmp = global_img_buf_.img_buf[0].img.clone();
    face_ptr_->GetRecognitionResult(mat_tmp, endlib_feats, match_info);
    if (match_info.size() > 0 && match_info[0].match_score > 0.9) {
      RCLCPP_INFO(this->get_logger(), "match info name:%s\n", match_info[0].face_id.c_str());
      break;
    }

  }
  return 0;

}

int VisionManager::addFaceInfo(std::string & args)
{
  std::string name;
  bool is_host = false;

  // parse command arguments
  std::map<std::string, std::string> params_map = parse_parameters(args);
  std::map<std::string, std::string>::iterator it;
  for (it = params_map.begin(); it != params_map.end(); it++) {
    if (it->first == "id") {
      name = it->second;
    }
    if (it->first == "host" && it->second == "true") {
      is_host = true;
    }
  }

  RCLCPP_INFO(this->get_logger(), "add face info id:%s host:%d", name.c_str(), is_host);

  if (name.length() == 0) {
    return -1;
  }

  FaceManager::getInstance()->addFaceIDCacheInfo(name, is_host);

  std::thread faceDet = std::thread(&VisionManager::FaceDetProc, this, name);
  faceDet.detach();

  return 0;
}


int VisionManager::cancelAddFace()
{
  return FaceManager::getInstance()->cancelAddFace();
}

int VisionManager::confirmFace(std::string & args)
{
  std::string name;
  bool is_host = false;

  // parse command arguments
  std::map<std::string, std::string> params_map = parse_parameters(args);
  std::map<std::string, std::string>::iterator it;
  for (it = params_map.begin(); it != params_map.end(); it++) {
    if (it->first == "id") {
      name = it->second;
    }
    if (it->first == "host" && it->second == "true") {
      is_host = true;
    }
  }

  RCLCPP_INFO(this->get_logger(), "id:%s host:%d", name.c_str(), is_host);

  if (name.length() == 0) {
    return -1;
  }

  return FaceManager::getInstance()->confirmFace(name, is_host);
}

int VisionManager::updateFaceId(std::string & args)
{
  std::vector<std::string> names;

  split_string(args, names, ":");
  if (names.size() != 2) {
    return -1;
  }

  std::string ori_name = names[0];
  std::string new_name = names[1];

  return FaceManager::getInstance()->updateFaceId(ori_name, new_name);
}

int VisionManager::deleteFace(std::string & args)
{
  std::string face_name;
  //bool is_host = false;

  // parse command arguments
  std::map<std::string, std::string> params_map = parse_parameters(args);
  std::map<std::string, std::string>::iterator it;
  for (it = params_map.begin(); it != params_map.end(); it++) {
    if (it->first == "id") {
      face_name = it->second;
    }
    if (it->first == "host" && it->second == "true") {
      //is_host = true;
    }
  }

  return FaceManager::getInstance()->deleteFace(face_name);
}


int VisionManager::getAllFaces(std::shared_ptr<FaceManagerT::Response> response)
{
  std::string all_face_info = FaceManager::getInstance()->getAllFaces();
  response->msg = all_face_info;

  RCLCPP_INFO(this->get_logger(), "all face info:%s\n", all_face_info.c_str());
  return 0;
}

bool VisionManager::CallService(
  rclcpp::Client<CameraServiceT>::SharedPtr & client,
  const uint8_t & cmd, const std::string & args)
{
  auto req = std::make_shared<CameraServiceT::Request>();
  req->command = cmd;
  req->args = args;

  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1);
  while (!client->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
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

VisionManager::~VisionManager()
{
  if (img_proc_thread_->joinable()) {
    img_proc_thread_->join();
  }

  if (0 == DetachShm(shm_addr_)) {
    DelShm(shm_id_);
  }
}

}
