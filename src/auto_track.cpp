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

#include <string>
#include <memory>

#include "cyberdog_vision/auto_track.hpp"
#include "cyberdog_vision/common_type.hpp"

namespace cyberdog_vision
{

AutoTrack::AutoTrack(const std::string & model_path)
: gpu_id_(0), loss_th_(300), fail_count_(0), is_init_(false), is_lost_(false)
{
  std::cout << "===Init AutoTrack===" << std::endl;
  std::string backbone_path = model_path + "/test_backbone.onnx";
  std::string head_path = model_path + "/test_rpn.onnx";
  std::string reid_path = model_path + "/any_reid_v2_sim.onnx";
  tracker_ptr_ = std::make_shared<TRACKER::Tracker>(backbone_path, head_path, reid_path, gpu_id_);
}

bool AutoTrack::SetTracker(const cv::Mat & img, const cv::Rect & bbox)
{
  XMImage xm_img;
  ImgConvert(img, xm_img);
  fail_count_ = 0;
  bool is_success = tracker_ptr_->init(xm_img, bbox);
  if (is_success) {
    is_init_ = true;
    is_lost_ = false;
    std::cout << "set auto track success." << std::endl;
  }
  return is_success;
}

bool AutoTrack::Track(const cv::Mat & img, cv::Rect & bbox)
{
  if (!is_init_) {
    std::cout << "Please set tracker before auto track. " << std::endl;
    return false;
  }

  XMImage xm_img;
  ImgConvert(img, xm_img);
  tracker_ptr_->track(xm_img);
  TRACKER::TrackBox track_box = tracker_ptr_->getBox();
  if (!track_box.track_sucess) {
    fail_count_++;
    if (fail_count_ > loss_th_) {
      std::cout << "Object lost, please set tracker to restart. " << std::endl;
      is_init_ = false;
      is_lost_ = true;
    }
    return false;
  }
  bbox = track_box.rect & cv::Rect(0, 0, img.cols, img.rows);
  fail_count_ = 0;
  return true;
}

void AutoTrack::SetLossTh(int loss_th)
{
  loss_th_ = loss_th;
}

void AutoTrack::ResetTracker()
{
  is_init_ = false;
}

bool AutoTrack::GetLostStatus()
{
  return is_lost_;
}

AutoTrack::~AutoTrack()
{}

}  // namespace cyberdog_vision
