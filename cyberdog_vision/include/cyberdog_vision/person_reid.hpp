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

#ifndef CYBERDOG_VISION__PERSON_REID_HPP_
#define CYBERDOG_VISION__PERSON_REID_HPP_

#include <string>
#include <vector>

#include "ReIDToolAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

enum SimType
{
  kSimOne2One = 0,
  kSimOne2Group,
  kSimGroup2Group
};

class PersonReID
{
public:
  explicit PersonReID(const std::string & model_path);
  ~PersonReID();

  int SetTracker(const cv::Mat & img, const cv::Rect & body_box, std::vector<float> & reid_feat);
  int GetReIDInfo(
    const cv::Mat & img, const std::vector<InferBbox> & body_bboxes, int & id,
    cv::Rect & tracked);
  int GetFeatureLen();
  void ResetTracker();
  bool GetLostStatus();

private:
  int GetFeature(const cv::Mat & img, const cv::Rect & body_box, std::vector<float> & reid_feat);
  float GetSim(
    std::vector<float> & feat_det,
    std::vector<float> & feat_library, const SimType & sim_type);

  int gpu_id_;
  int tracking_id_;
  int object_loss_th_;
  int library_frame_num_;
  int unmatch_count_;
  float feat_sim_th_;
  float feat_update_th_;
  bool is_tracking_;
  bool is_lost_;

  void * reid_ptr_;
  std::vector<float> tracker_feat_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__PERSON_REID_HPP_
