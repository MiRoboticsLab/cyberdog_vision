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

#ifndef CYBERDOG_VISION__FACE_RECOGNITION_HPP_
#define CYBERDOG_VISION__FACE_RECOGNITION_HPP_

#include <string>
#include <vector>
#include <memory>
#include <map>

#include "XMFaceAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class FaceRecognition
{
public:
  FaceRecognition(
    const std::string & model_path, bool open_emotion, bool open_age);
  ~FaceRecognition();

  int GetFaceInfo(const cv::Mat & img, std::vector<EntryFaceInfo> & faces_info);
  int GetRecognitionResult(
    const cv::Mat & img, const std::map<std::string, std::vector<float>> & endlib_feats,
    std::vector<MatchFaceInfo> & faces_info);

private:
  void FillParam(const std::string & model_path, FaceParam & param);
  XMFaceAPI * face_ptr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__FACE_RECOGNITION_HPP_
