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

#include "cyberdog_vision/person_reid.hpp"

const int kFeatLen = 128;
namespace cyberdog_vision
{

PersonReID::PersonReID(const std::string & model_reid)
: gpu_id_(0), tracking_id_(0), object_loss_th_(300), library_frame_num_(15), unmatch_count_(0),
  feat_sim_th_(0.8), feat_update_th_(0.9), is_tracking_(false), reid_ptr_(nullptr)
{
  if (0 != REID_Init(reid_ptr_, model_reid.c_str(), gpu_id_)) {
    throw std::logic_error("Init person reid algo fial. ");
  }
}

int PersonReID::SetTracker(
  const cv::Mat & img, const cv::Rect & body_box,
  std::vector<float> & reid_feat)
{
  if (0 != GetFeature(img, body_box, reid_feat)) {
    std::cout << "GetFeature fail. " << std::endl;
    return -1;
  }
  tracker_feat_.assign(reid_feat.begin(), reid_feat.end());
  if (is_tracking_) {
    tracking_id_++;
  }
  is_tracking_ = true;

  return 0;
}

int PersonReID::GetReIDInfo(
  const cv::Mat & img, const std::vector<InferBbox> & body_bboxes,
  int & id, size_t & index)
{
  if (tracker_feat_.empty()) {
    std::cout << "Please set tracker before tracking. " << std::endl;
    return -1;
  }

  id = -1;
  double max_sim = 0;
  std::vector<float> match_feat;
  for (size_t i = 0; i < body_bboxes.size(); ++i) {
    // For every body bbox
    std::vector<float> feat;
    if (0 != GetFeature(img, body_bboxes[i].body_box, feat)) {
      return -1;
    }
    if (0 != tracker_feat_.size()) {
      double sim_val = GetSim(feat, tracker_feat_, SimType::kSimOne2Group);
      std::cout << "Object " << i << " sim: " << sim_val << std::endl;
      if (sim_val > max_sim) {
        index = i;
        max_sim = sim_val;
        match_feat.assign(feat.begin(), feat.end());
      }
    } else {
      tracker_feat_.assign(feat.begin(), feat.end());
    }
  }

  if (max_sim > feat_sim_th_) {
    // Match success
    std::cout << "Match success, sim: " << max_sim << std::endl;
    unmatch_count_ = 0;
    id = tracking_id_;
    if (max_sim > feat_update_th_) {
      // Update library feat
      if ((int)tracker_feat_.size() / kFeatLen == library_frame_num_) {
        tracker_feat_.erase(tracker_feat_.begin(), tracker_feat_.begin() + kFeatLen);
      }
      tracker_feat_.insert(tracker_feat_.end(), match_feat.begin(), match_feat.end());
    }
  } else {
    std::cout << "Match fail, current count: " << unmatch_count_ << std::endl;
    unmatch_count_++;
    if (unmatch_count_ > object_loss_th_) {
      std::cout << "Object is lost. " << std::endl;
      ResetTracker();
    }
  }
  return 0;
}

int PersonReID::GetFeatureLen()
{
  return REID_GetFeatLen();
}

void PersonReID::ResetTracker()
{
  is_tracking_ = false;
  tracker_feat_.clear();
  tracking_id_++;
}

int PersonReID::GetFeature(
  const cv::Mat & img, const cv::Rect & body_box,
  std::vector<float> & reid_feat)
{
  cv::Mat reid_img = img(body_box);
  XMReIDImage xm_reid_img;
  xm_reid_img.data = reid_img.data;
  xm_reid_img.height = reid_img.rows;
  xm_reid_img.width = reid_img.cols;
  xm_reid_img.fmt = XM_IMG_FMT_BGR;
  float * feat = nullptr;
  if (0 != REID_ExtractFeat(reid_ptr_, &xm_reid_img, feat)) {
    std::cout << "Extract reid feature fail." << std::endl;
    return -1;
  }
  reid_feat.resize(GetFeatureLen());
  if (feat != nullptr) {
    memcpy(reid_feat.data(), feat, sizeof(float) * GetFeatureLen());
    feat = nullptr;
  }
  return 0;
}

float PersonReID::GetSim(
  std::vector<float> & feat_det, std::vector<float> & feat_library,
  const SimType & sim_type)
{
  double sim_value;
  switch (sim_type) {
    case SimType::kSimOne2One:
      sim_value = REID_GetSimOfOne2One(reid_ptr_, feat_det.data(), feat_library.data());
      break;
    case SimType::kSimOne2Group:
      sim_value = REID_GetSimOfOne2Group(
        reid_ptr_, feat_det.data(),
        feat_library.data(), feat_library.size() / kFeatLen);
      break;
    case SimType::kSimGroup2Group:
      sim_value = REID_GetSimOfGroup2Group(
        reid_ptr_, feat_det.data(),
        feat_det.size() / kFeatLen, feat_library.data(), feat_library.size() / kFeatLen);
      break;
  }
  return sim_value;
}

PersonReID::~PersonReID()
{
  if (reid_ptr_) {
    REID_Release(reid_ptr_);
  }
}

}  // namespace cyberdog_vision
