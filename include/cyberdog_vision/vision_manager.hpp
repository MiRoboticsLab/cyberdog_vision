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

#ifndef CYBERDOG_VISION__VISION_MANAGER_HPP_
#define CYBERDOG_VISION__VISION_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>

#include "cyberdog_vision/shared_memory_op.hpp"

namespace cyberdog_vision
{

class VisionManager : public rclcpp::Node
{
public:
  VisionManager();
  ~VisionManager();

private:
  int Init();
  void ImageProc();

private:
  std::vector<cv::Mat> img_buf_;
  std::shared_ptr<std::thread> img_proc_thread_;

  int shm_id_;
  int sem_set_id_;
  char * shm_addr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__VISION_MANAGER_HPP_
