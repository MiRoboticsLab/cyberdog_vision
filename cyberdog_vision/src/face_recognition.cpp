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
#include <vector>
#include <map>

#include "cyberdog_vision/face_recognition.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog_vision
{

FaceRecognition::FaceRecognition(
  const std::string & model_path, bool open_emotion, bool open_age)
{
  INFO("===Init FaceRecognition===");
  face_ptr_ = XMFaceAPI::Create();

  FaceParam param;
  FillParam(model_path, param);
  param.open_emotion = open_emotion;
  param.open_age = open_age;
  param.det_scale = 512;
  param.feat_thres = 0.65;
  if (!face_ptr_->init(param)) {
    throw std::logic_error("Init face recognition algo fial. ");
  }

  std::string version;
  if (!face_ptr_->getVersion(version)) {
    INFO("Version of face sdk is %s", version.c_str());
  }
}

int FaceRecognition::GetFaceInfo(const cv::Mat & img, std::vector<EntryFaceInfo> & faces_info)
{
  XMImage xm_img;
  ImgConvert(img, xm_img);
  if (!face_ptr_->getFaceInfo(xm_img, faces_info)) {
    return -1;
  }
  return 0;
}

int FaceRecognition::GetRecognitionResult(
  const cv::Mat & img, const std::map<std::string,
  std::vector<float>> & endlib_feats,
  std::vector<MatchFaceInfo> & faces_info)
{
  XMImage xm_img;
  ImgConvert(img, xm_img);
  if (!face_ptr_->getMatchInfo(xm_img, endlib_feats, faces_info)) {
    return -1;
  }
  return 0;
}

void FaceRecognition::FillParam(const std::string & model_path, FaceParam & param)
{
  param.detect_mf = model_path + "/detect/mnetv2_gray_nop_light_epoch_235_512.onnx";
  param.lmk_mf = model_path + "/landmark/groupdiv128a_vis_uncertain_sim.onnx";
  param.feat_mf = model_path + "/feature/airfacenet_v1.onnx";
  param.emotion_mf = model_path + "/emotion/speaker_v5_island_v4_data3_sim.onnx";
  param.age_mf = model_path + "/age/tinyage_gray_112_age_gender_sim.onnx";
}

FaceRecognition::~FaceRecognition()
{
  if (face_ptr_ != nullptr) {
    XMFaceAPI::Destroy(face_ptr_);
  }
}

}  // namespace cyberdog_vision
