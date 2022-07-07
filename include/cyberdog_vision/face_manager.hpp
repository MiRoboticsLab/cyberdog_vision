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


#ifndef CYBERDOG_VISION__FACE_MANAGER_HPP_
#define CYBERDOG_VISION__FACE_MANAGER_HPP_

#include "cyberdog_vision/face_recognition.hpp"

namespace cyberdog_vision
{

struct FaceId
{
  std::string name;
  bool is_host;
};

class FaceManager
{
public:
  static FaceManager * getInstance();
  static const std::string getFaceDataPath();
  std::map<std::string, std::vector<float>> & getFeatures();
  int addFaceIDCacheInfo(std::string & name,bool is_host);
  int addFaceFeatureCacheInfo(std::vector<EntryFaceInfo> & faceinfo);
  int cancelAddFace();
  bool checkFacePose(std::vector<EntryFaceInfo> &faceinfo);
  int confirmFace(std::string & name,bool is_host);
  int updateFaceId(std::string & ori_name, std::string & new_name);
  int deleteFace(std::string & face_name);
  std::string getAllFaces();
  bool findFace(const std::string & face_name);
  bool isHost(const std::string & face_name);

private:
  FaceManager();
  ~FaceManager();
  void initialize();
  bool loadFeatures();

  std::map<std::string, std::vector<float>> m_features;
  std::map<std::string, bool> m_hostMap;
  std::mutex m_mutex;

  bool m_inFaceAdding;
  /* save last face info, wait for user confirm */
  FaceId m_faceIdCached;
  std::vector<float> m_faceFeatsCached;

};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__FACE_MANAGER_HPP_

