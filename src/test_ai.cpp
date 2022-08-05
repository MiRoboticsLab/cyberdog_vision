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

#include <string>
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

#include "cyberdog_vision/body_detection.hpp"
#include "cyberdog_vision/face_recognition.hpp"
#include "cyberdog_vision/gesture_recognition.hpp"
#include "cyberdog_vision/keypoints_detection.hpp"
#include "cyberdog_vision/person_reid.hpp"

const char kPathPrefix[] = "/SDCARD/vision";

int test_body(const std::vector<cv::String> & file_names)
{
  std::cout << "===test_body===" << std::endl;
  cyberdog_vision::BodyDetection body_ = cyberdog_vision::BodyDetection(
    kPathPrefix + std::string("/body_gesture"));
  for (size_t i = 0; i < file_names.size(); i++) {
    // For every image
    std::cout << file_names[i] << std::endl;
    cv::Mat img = cv::imread(file_names[i]);
    cyberdog_vision::BodyFrameInfo infos;
    double tim1 = static_cast<double>(cv::getTickCount());
    if (0 != body_.Detect(img, infos)) {
      return -1;
    }
    double tim2 = static_cast<double>(cv::getTickCount()) - tim1;
    std::cout << "body detection time:" << float(tim2 / cv::getTickFrequency()) << std::endl;
    std::cout << "person num: " << infos.size() << std::endl;
    for (size_t j = 0; j < infos.size(); ++j) {
      std::cout << "Person : " << j << std::endl;
      std::cout << "l-w-h" << infos[j].top << ", " << infos[j].left << ", " << infos[j].width <<
        ", " << infos[j].height << std::endl;
      std::cout << "score: " << infos[j].score << std::endl;
    }
  }
  return 0;
}

int test_face(const std::vector<cv::String> & file_names)
{
  std::cout << "===test_face===" << std::endl;
  cyberdog_vision::FaceRecognition face_ = cyberdog_vision::FaceRecognition(
    kPathPrefix + std::string("/face_recognition"), true, true);
  std::cout << "Init complate. " << std::endl;
  for (size_t i = 0; i < file_names.size(); i++) {
    // For every image
    std::cout << file_names[i] << std::endl;
    cv::Mat img = cv::imread(file_names[i]);
    std::vector<EntryFaceInfo> faces_info;
    if (0 != face_.GetFaceInfo(img, faces_info)) {
      return -1;
    }
    for (auto & info : faces_info) {
      std::cout << "lmk: " << std::endl;
      for (auto & lmk : info.lmks) {
        std::cout << lmk << ", ";
      }
      std::cout << std::endl;
      std::cout << "pose " << std::endl;
      for (auto & pose : info.poses) {
        std::cout << pose << ",";
      }
      std::cout << std::endl;
      std::cout << "feats " << std::endl;
      for (auto & feat : info.feats) {
        std::cout << feat << ",";
      }
      std::cout << std::endl;
      std::cout << "emo: " << std::endl;
      for (auto & emo : info.emotions) {
        std::cout << emo << ",";
      }
      std::cout << std::endl;
    }
  }
  return 0;
}

int test_gesture(const std::vector<cv::String> & file_names)
{
  std::cout << "===test_gesture===" << std::endl;
  cyberdog_vision::BodyDetection body_ = cyberdog_vision::BodyDetection(
    kPathPrefix + std::string("/body_gesture"));
  cyberdog_vision::GestureRecognition gesture_ = cyberdog_vision::GestureRecognition(
    kPathPrefix + std::string("/body_gesture"));

  for (size_t i = 0; i < file_names.size(); i++) {
    // For every image
    std::cout << file_names[i] << std::endl;
    cv::Mat img = cv::imread(file_names[i]);
    // Body detection
    cyberdog_vision::BodyFrameInfo bodies;
    if (0 != body_.Detect(img, bodies)) {
      return -1;
    }
    std::vector<cyberdog_vision::InferBbox> infer_bboxes = cyberdog_vision::BodyConvert(bodies);
    // Gesture recognition
    std::vector<cyberdog_vision::GestureInfo> infos;
    gesture_.GetGestureInfo(img, infer_bboxes, infos);
    std::cout << "Person num: " << infos.size() << std::endl;
    for (size_t i = 0; i < infos.size(); ++i) {
      std::cout << "Person " << i << ": label - rect" << std::endl;
      std::cout << infos[i].label << std::endl;
      std::cout << infos[i].rect << std::endl;
    }
  }
  return 0;
}

int test_keypoints(const std::vector<cv::String> & file_names)
{
  std::cout << "===test_keypoints===" << std::endl;
  cyberdog_vision::BodyDetection body_ = cyberdog_vision::BodyDetection(
    kPathPrefix + std::string("/body_gesture"));
  cyberdog_vision::KeypointsDetection keypoints_ = cyberdog_vision::KeypointsDetection(
    kPathPrefix + std::string("/keypoints_detection"));
  std::cout << "Init complate. " << std::endl;
  for (size_t i = 0; i < file_names.size(); i++) {
    // For every image
    std::cout << file_names[i] << std::endl;
    cv::Mat img = cv::imread(file_names[i]);
    // Body detection
    cyberdog_vision::BodyFrameInfo bodies;
    if (0 != body_.Detect(img, bodies)) {
      return -1;
    }
    std::cout << "detect person num: " << bodies.size() << std::endl;

    // Get keypoints
    std::vector<std::vector<cv::Point2f>> bodies_keypoints;
    std::vector<cyberdog_vision::InferBbox> infer_bboxes = cyberdog_vision::BodyConvert(bodies);
    keypoints_.GetKeypointsInfo(img, infer_bboxes, bodies_keypoints);
    for (auto & box : infer_bboxes) {
      std::cout << box.body_box << std::endl;
    }

    for (size_t m = 0; m < bodies_keypoints.size(); ++m) {
      for (size_t n = 0; n < bodies_keypoints[m].size(); ++n) {
        std::cout << bodies_keypoints[m][n].x << ", " << bodies_keypoints[m][n].y << std::endl;
      }
    }
  }
  return 0;
}

int test_reid(const std::vector<cv::String> & file_names)
{
  std::cout << "===test_reid===" << std::endl;
  cyberdog_vision::PersonReID reid_ = cyberdog_vision::PersonReID(
    kPathPrefix + std::string("/person_reid"));
  for (size_t i = 0; i < file_names.size(); i++) {
    // For every image
    std::cout << i << " name: " << file_names[i] << std::endl;
    cv::Mat img = cv::imread(file_names[i]);
    std::vector<float> reid_feat;
    cv::Rect rect = cv::Rect(0, 0, img.cols, img.rows);
    if (0 != reid_.SetTracker(img, rect, reid_feat)) {
      std::cout << "Fail. " << std::endl;
      return -1;
    }
    std::cout << "extract feat complate. " << std::endl;
  }
  return 0;
}

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cout << "Err: not enough param. " << std::endl;
    return -1;
  }
  // Read image from folder
  std::string folder_path = argv[2];
  std::vector<cv::String> file_names;
  cv::glob(folder_path.c_str(), file_names);

  if (0 == std::strcmp(argv[1], "body")) {
    test_body(file_names);
  } else if (0 == std::strcmp(argv[1], "face")) {
    test_face(file_names);
  } else if (0 == std::strcmp(argv[1], "gesture")) {
    test_gesture(file_names);
  } else if (0 == std::strcmp(argv[1], "keypoints")) {
    test_keypoints(file_names);
  } else if (0 == std::strcmp(argv[1], "reid")) {
    test_reid(file_names);
  }

  return 0;
}
