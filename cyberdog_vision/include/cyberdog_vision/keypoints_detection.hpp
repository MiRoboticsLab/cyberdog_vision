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

#ifndef CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_
#define CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_

#include <string>
#include <vector>
#include <memory>

#include "person_keypoints.h"  // NOLINT
#include "common_type.hpp"

namespace cyberdog_vision
{

class KeypointsDetection
{
public:
  explicit KeypointsDetection(const std::string & model_path);
  ~KeypointsDetection();

  void GetKeypointsInfo(
    const cv::Mat & img, const std::vector<InferBbox> & body_boxes,
    std::vector<std::vector<cv::Point2f>> & bodies_keypoints);

private:
  std::shared_ptr<Person_keyPoints> keypoints_ptr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_
