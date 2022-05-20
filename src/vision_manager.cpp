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

#include <stdlib.h>

#include "cyberdog_vision/vision_manager.hpp"
#include "cyberdog_vision/semaphore_op.hpp"

#define SHM_PROJ_ID 'A'
#define SEM_PROJ_ID 'B'

namespace cyberdog_vision
{

VisionManager::VisionManager()
: Node("vision_manager"), shm_addr_(nullptr)
{
  if (0 != Init()) {
    throw std::logic_error("Init shared memory or semaphore fail. ");
  }

  // Create process thread
  img_proc_thread_ = std::make_shared<std::thread>(&VisionManager::ImageProc, this);
}

int VisionManager::Init()
{
  // Create shared memory and get address
  if (0 != CreateShm(SHM_PROJ_ID, sizeof(double) + IMAGE_SIZE, shm_id_)) {
    return -1;
  }
  shm_addr_ = GetShmAddr(shm_id_, sizeof(double) + IMAGE_SIZE);
  if (shm_addr_ == nullptr) {
    return -1;
  }

  // Create semaphore set, 0-mutex, 1-empty, 2-full
  if (0 != CreateSem(SEM_PROJ_ID, 3, sem_set_id_)) {
    return -1;
  }

  // Set init value of the semaphore
  if (0 != SetSemInitVal(sem_set_id_, 0, 1)) {
    return -1;
  }

  if (0 != SetSemInitVal(sem_set_id_, 1, 1)) {
    return -1;
  }

  if (0 != SetSemInitVal(sem_set_id_, 2, 0)) {
    return -1;
  }

  return 0;
}

void VisionManager::ImageProc()
{
  while (rclcpp::ok()) {
    WaitSem(sem_set_id_, 2);
    WaitSem(sem_set_id_, 0);
    cv::Mat img;
    img.create(480, 640, CV_8UC3);
    memcpy(img.data, (char *)shm_addr_ + sizeof(double), IMAGE_SIZE);
    SignalSem(sem_set_id_, 0);
    SignalSem(sem_set_id_, 1);
  }
  if (0 == DetachShm(shm_addr_)) {
    DelShm(shm_id_);
  }
}

VisionManager::~VisionManager()
{
  if (img_proc_thread_->joinable()) {
    img_proc_thread_->join();
  }
}

}
