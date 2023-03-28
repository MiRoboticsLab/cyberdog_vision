// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include "cyberdog_vision/body_detection.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog_vision
{

BodyDetection::BodyDetection(const std::string & model_path)
: gpu_id_(0)
{
  INFO("===Init BodyDetection===");
  std::string model_det = model_path + "/detect.onnx";
  std::string model_cls = model_path + "/cls_human_mid.onnx";
  body_ptr_ = std::make_shared<ContentMotionAPI>();
  if (0 != body_ptr_->Init(model_det, "", model_cls, gpu_id_)) {
    throw std::logic_error("Init body detection algo fial. ");
  }
}

int BodyDetection::Detect(const cv::Mat & img, BodyFrameInfo & infos)
{
  if (img.empty()) {
    WARN("Image is empty cannot perform detection. ");
    return -1;
  }

  XMImage xm_img;
  ImgConvert(img, xm_img);
  struct LogInfo log_info;
  if (0 != body_ptr_->GetContentMotionAnalyse(xm_img, infos, log_info, gpu_id_)) {
    WARN("Detacte body fail. ");
    return -1;
  }

  return 0;
}

BodyDetection::~BodyDetection()
{
  if (body_ptr_ != nullptr) {
    body_ptr_->Close();
  }
}

}  // namespace cyberdog_vision
