// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef CYBERDOG_VISION__AUTO_TRACK_HPP_
#define CYBERDOG_VISION__AUTO_TRACK_HPP_

#include <string>
#include <memory>

#include "tracker.hpp"

namespace cyberdog_vision
{

class AutoTrack
{
public:
  explicit AutoTrack(const std::string & model_path);
  ~AutoTrack();

  bool SetTracker(const cv::Mat & img, const cv::Rect & bbox);
  bool Track(const cv::Mat & img, cv::Rect & bbox);
  void SetLossTh(int loss_th);
  void ResetTracker();
  bool GetLostStatus();

private:
  std::shared_ptr<TRACKER::Tracker> tracker_ptr_;

  int gpu_id_;
  int loss_th_;
  int fail_count_;
  bool is_init_;
  bool is_lost_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__AUTO_TRACK_HPP_
