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

#ifndef CYBERDOG_VISION__AUTO_TRACK_HPP_
#define CYBERDOG_VISION__AUTO_TRACK_HPP_

#include "tracker.hpp"

namespace cyberdog_vision
{

class AutoTrack
{
public:
  AutoTrack(const std::string & model_path);
  ~AutoTrack();

  int SetTracker(const cv::Mat & img, const cv::Rect & bbox);
  bool Track(const cv::Mat & img, cv::Rect & bbox);
  void SetLossTh(int loss_th);

private:
  std::shared_ptr<TRACKER::Tracker> tracker_ptr_;
  int gpu_id_;
  int loss_th_;
  int fail_count_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__AUTO_TRACK_HPP_
