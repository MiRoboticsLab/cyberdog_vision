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

#include "cyberdog_vision/face_recognition.hpp"

namespace cyberdog_vision
{

FaceRecognition::FaceRecognition(
  const std::string & model_det, const std::string & model_lmk,
  const std::string & model_feat, const std::string & model_emotion,
  bool open_emotion)
{
  face_ptr_ = XMFaceAPI::Create();

  FaceParam param;
  param.detect_mf = model_det;
  param.lmk_mf = model_lmk;
  param.feat_mf = model_feat;
  param.emotion_mf = model_emotion;
  param.open_emotion = open_emotion;
  param.det_scale = 512;
  param.feat_thres = 0.65;
  if (!face_ptr_->init(param)) {
    throw std::logic_error("Init face recognition algo fial. ");
  }
  std::string version;
  if (!face_ptr_->getVersion(version)) {
    std::cout << "Version of face sdk is " << version << std::endl;
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

FaceRecognition::~FaceRecognition()
{
  if (face_ptr_ != nullptr) {
    XMFaceAPI::Destroy(face_ptr_);
  }
}

}  // namespace cyberdog_vison
