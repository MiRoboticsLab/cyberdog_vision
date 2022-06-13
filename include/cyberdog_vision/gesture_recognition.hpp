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

#ifndef CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_
#define CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_

#include "person_detect_hand_gesture.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class GestureRecognition
{
public:
  GestureRecognition(const std::string & model_det, const std::string & model_cls);
  ~GestureRecognition();

  int GetGestureInfo(
    const cv::Mat & img, const std::vector<InferBbox> & body_boxes,
    int & gesture_cls, cv::Rect & gesture_box);

private:
  std::shared_ptr<Hand_Gesture_Classification_Node> gesture_ptr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_
