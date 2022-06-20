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

const std::string kModelPath = "/SDCARD/ros2_ws/src/cyberdog_vision/3rdparty/";
namespace cyberdog_vision
{

VisionManager::VisionManager()
: Node("vision_manager"), shm_addr_(nullptr), buff_size_(6), is_tracking_(false)
{
  if (0 != Init()) {
    throw std::logic_error("Init shared memory or semaphore fail. ");
  }

  // Create process thread
  img_proc_thread_ = std::make_shared<std::thread>(&VisionManager::ImageProc, this);
  body_det_thread_ = std::make_shared<std::thread>(&VisionManager::BodyDet, this);
  reid_thread_ = std::make_shared<std::thread>(&VisionManager::ReIDProc, this);

  // Create AI object
  body_ptr_ = std::make_shared<BodyDetection>(
    kModelPath + "body_gesture/model/detect.onnx",
    kModelPath + "body_gesture/model/cls_human_mid.onnx");

  reid_ptr_ = std::make_shared<PersonReID>(kModelPath + "person_reid/model/reid_v1_mid.engine");

  // Create service
  tracking_service_ = create_service<BodyRegionT>(
    "tracking_object", std::bind(
      &VisionManager::TrackingService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create publisher
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  body_pub_ = create_publisher<BodyFrameInfoT>("body", pub_qos);
}

int VisionManager::Init()
{
  // Create shared memory and get address
  if (0 != CreateShm(SHM_PROJ_ID, sizeof(double) + IMAGE_SIZE, shm_id_)) {
    return -1;
  }
  shm_addr_ = GetShmAddr(shm_id_, sizeof(double) + IMAGE_SIZE);
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

void VisionManager::ImageProc()
{
  while (rclcpp::ok()) {
    // TODO: add timestamp
    WaitSem(sem_set_id_, 2);
    WaitSem(sem_set_id_, 0);
    StampedImage simg;
    simg.img.create(480, 640, CV_8UC3);
    memcpy(simg.img.data, (char *)shm_addr_ + sizeof(double), IMAGE_SIZE);
    SignalSem(sem_set_id_, 0);
    SignalSem(sem_set_id_, 1);

    // Save image to buffer, only process with real img
    std::unique_lock<std::mutex> lk(global_img_buf_.mtx);
    global_img_buf_.img_buf.clear();
    global_img_buf_.img_buf.push_back(simg);
  }
}

void VisionManager::BodyDet()
{
  while (rclcpp::ok()) {
    std::lock(global_img_buf_.mtx, body_results_.mtx);
    std::unique_lock<std::mutex> lk_img(global_img_buf_.mtx, std::adopt_lock);
    std::unique_lock<std::mutex> lk_body(body_results_.mtx, std::adopt_lock);
    if (!global_img_buf_.img_buf.empty()) {
      BodyFrameInfo infos;
      if (-1 != body_ptr_->Detect(global_img_buf_.img_buf[0].img, infos)) {
        if (body_results_.body_infos.size() >= buff_size_) {
          body_results_.body_infos.erase(body_results_.body_infos.begin());
        }
        // debug - visualization
        // std::cout << "Detection result: " << std::endl;
        // std::cout << "Person num: " << infos.size() << std::endl;
        // cv::Mat img_show = global_img_buf_.img_buf[0].img.clone();
        // for (auto & res : infos) {
        //   cv::rectangle(
        //     img_show, cv::Rect(res.left, res.top, res.width, res.height),
        //     cv::Scalar(0, 0, 255));
        // }
        // cv::imshow("img", img_show);
        // cv::waitKey(10);

        body_results_.body_infos.push_back(infos);
        body_results_.detection_img.img = global_img_buf_.img_buf[0].img.clone();
        body_results_.cond.notify_one();
      } else {
        RCLCPP_WARN(get_logger(), "Body detect fail of current image. ");
      }
    }
  }
}

void VisionManager::ReIDProc()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(body_results_.mtx);
    body_results_.cond.wait(lk, [this] {return !body_results_.body_infos.empty();});
    int person_id = -1;
    size_t person_index;
    std::vector<InferBbox> body_bboxes = BodyConvert(body_results_.body_infos[0]);
    if (-1 !=
      reid_ptr_->GetReIDInfo(
        body_results_.detection_img.img, body_bboxes, person_id,
        person_index) &&
      -1 != person_id)
    {
      RCLCPP_INFO(get_logger(), "Reid result, person id: %d", person_id);
    }
    PublishResult(body_results_, person_id, person_index);
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

void VisionManager::PublishResult(
  const BodyResults & body_results, int & person_id,
  size_t & person_index)
{
  BodyFrameInfoT pub_infos;
  pub_infos.header = body_results.detection_img.header;
  BodyFrameInfo curr_frame = body_results.body_infos[body_results.body_infos.size() - 1];
  pub_infos.count = curr_frame.size();
  for (size_t i = 0; i < pub_infos.count; ++i) {
    BodyInfoT body;
    body.roi.x_offset = curr_frame[i].left;
    body.roi.y_offset = curr_frame[i].top;
    body.roi.width = curr_frame[i].width;
    body.roi.height = curr_frame[i].height;
    if (-1 != person_id && i == person_index) {
      body.reid = std::to_string(person_id);
    }
    pub_infos.infos.push_back(body);
  }
  body_pub_->publish(pub_infos);
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
