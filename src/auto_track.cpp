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


#include "cyberdog_vision/auto_focus.hpp"
#include "cyberdog_vision/common_type.hpp"

namespace cyberdog_vision
{

AutoTrack::AutoTrack(const std::string & model_path)
: gpu_id_(0), loss_th_(300), fail_count_(0)
{
  std::string backbone_path = model_path + "/model/test_backbone.onnx";
  std::string head_path = model_path + "/model/test_rpn.onnx";
  std::string reid_path = model_path + "/model/any_reid_v2_sim.onnx";
  tracker_ptr_ = std::make_shared<TRACKER::Tracker>(backbone_path, head_path, reid_path, gpu_id_);
}

int AutoTrack::SetTracker(const cv::Mat & img, const cv::Rect & bbox)
{
  XMImage xm_img;
  ImgConvert(img, xm_img);
  fail_count_ = 0;
  return tracker_ptr_->init(xm_img, bbox);
}

bool AutoTrack::Track(const cv::Mat & img, cv::Rect & bbox)
{
  XMImage xm_img;
  ImgConvert(img, xm_img);
  tracker_ptr_->track(xm_img);
  TrackBox track_box = tracker_ptr_->getBox();
  if (!track_box.track_sucess) {
    fail_count_++;
    return false;
  }
  fail_count_ = 0;
  return true;
}

void AutoTrack::SetLossTh(int loss_th)
{
  loss_th_ = loss_th;
}

AutoTrack::~AutoTrack()
{}

} // namespace cyberdog_vision
