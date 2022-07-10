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

#ifndef CYBERDOG_VISION__COMMON_TYPE_HPP_
#define CYBERDOG_VISION__COMMON_TYPE_HPP_

#include <string>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>

#include <std_msgs/msg/header.hpp>

#include "XMFaceAPI.h"
#include "ContentMotionAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

using BodyFrameInfo = std::vector<HumanBodyInfo>;

struct InferBbox
{
  cv::Rect body_box;
  float score;
};

struct GestureInfo
{
  cv::Rect rect;
  int label;
};

struct StampedImage
{
  std_msgs::msg::Header header;
  cv::Mat img;
};

struct GlobalImageBuf
{
  bool is_filled;
  std::mutex mtx;
  std::condition_variable cond;
  std::vector<StampedImage> img_buf;
  GlobalImageBuf()
  {
    is_filled = false;
  }
};

struct BodyResults
{
  bool is_filled;
  std::mutex mtx;
  std::condition_variable cond;
  StampedImage detection_img;
  std::vector<BodyFrameInfo> body_infos;
  BodyResults()
  {
    is_filled = false;
  }
};

struct AlgoStruct
{
  bool is_called;
  std::mutex mtx;
  std::condition_variable cond;
  AlgoStruct()
  {
    is_called = false;
  }
};

struct AlgoProcess
{
  int process_num;
  std::mutex mtx;
  std::condition_variable cond;
  AlgoProcess()
  {
    process_num = 0;
  }
};

inline void ImgConvert(const cv::Mat & img, XMImage & xm_img)
{
  xm_img.data = img.data;
  xm_img.width = img.cols;
  xm_img.height = img.rows;
  xm_img.channel = img.channels();
  xm_img.type = ColorType::BGR;
}

inline std::vector<InferBbox> BodyConvert(BodyFrameInfo & infos)
{
  std::vector<InferBbox> infer_bboxes;
  for (auto & info : infos) {
    InferBbox box;
    box.body_box = cv::Rect(info.left, info.top, info.width, info.height);
    box.score = info.score;
    infer_bboxes.push_back(box);
  }
  return infer_bboxes;
}

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__COMMON_TYPE_HPP_
