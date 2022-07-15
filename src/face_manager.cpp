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

#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>
#include <utility>
#include <numeric>

#include "cyberdog_vision/face_manager.hpp"

namespace cyberdog_vision
{

static const char * label_path = "/home/mi/.faces/faceinfo.yaml";
// key value to judge whether face is legal or not.
static const float FACE_NUMBER_STABLE_VAL = 0.0f;
static const float FACE_POSE_STABLE_THRES = 3.0f;
static const float FACE_POSE_YAW_LEGAL_THRES = 30.0f;
static const float FACE_POSE_PITCH_LEGAL_THRES = 20.0f;
static const float FACE_POSE_ROW_LEGAL_THRES = 30.0f;
static const float FACE_AREA_STABLE_THRES = 0.0010f;
static const float FACE_AREA_LEGAL_THRES = 0.005f;

void get_mean_stdev(std::vector<float> & vec, float & mean, double & stdev)
{
  size_t count = vec.size();
  float sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  mean = sum / count;

  double accum = 0.0;
  for (size_t i = 0; i < count; i++) {
    accum += (vec[i] - mean) * (vec[i] - mean);
  }

  stdev = sqrt(accum / count);
}

FaceManager * FaceManager::getInstance()
{
  static FaceManager s_instance;
  return &s_instance;
}

FaceManager::FaceManager()
{
  m_inFaceAdding = false;
  initialize();
}

FaceManager::~FaceManager()
{
}

const std::string FaceManager::getFaceDataPath()
{
  return label_path;
}

void FaceManager::initialize()
{
  if (!loadFeatures()) {
    std::cout << "Failed to load face features." << std::endl;
  }
}

std::map<std::string, std::vector<float>> & FaceManager::getFeatures()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  return m_features;
}

bool FaceManager::updateFeaturesFile()
{
  std::map<std::string, std::vector<float>>::iterator feature_iter;

  string faceinfofile = std::string(label_path);
  string face_name;
  bool is_host;
  vector<float> face_feature;
  cv::FileStorage fs(faceinfofile, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
      cout << "cannot open xml file to write" << endl;
      return false;
  }
  fs << "UserFaceInfo" << "[";
  for (feature_iter = m_features.begin();feature_iter != m_features.end();feature_iter++)
  {
    //std::cout << feature_iter->first << " : " << feature_iter->second << std::endl;
    face_name = feature_iter->first;
    face_feature = feature_iter->second;
    is_host = m_hostMap[face_name];
    fs << "{" << "name" << face_name << "is_host" << is_host << "feature" << face_feature << "}";
  }
  fs << "]";
  fs.release();
  return true;
}

bool FaceManager::loadFeatures()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  if (access(label_path, 0) != 0) {
    std::cout << "faces path not found." << std::endl;
    umask(0);
    return true;
  }

  std::string faceinfofile = std::string(label_path);
  int is_host;
  std::string face_name;
  vector<float> face_feature;

  cv::FileStorage file_read(faceinfofile, cv::FileStorage::READ);
  if (!file_read.isOpened())
  {
    cout << "cannot open yaml file to write" << endl;
    return false;
  }

  cv::FileNode UserFaceInfo = file_read["UserFaceInfo"];
  for(cv::FileNodeIterator it = UserFaceInfo.begin(); it != UserFaceInfo.end(); ++it){
    (*it)["name"] >> face_name;
    (*it)["feature"] >> face_feature;
    (*it)["is_host"] >> is_host;

    m_features[face_name] = face_feature;
    m_hostMap[face_name] = is_host;

    std::cout << "Load known face info " << face_name << "host:" << is_host << std::endl;
  }
  file_read.release();

  return true;
}

int FaceManager::checkFacePose(std::vector<EntryFaceInfo> &faceinfos,std::string &msg)
{
  float mean[statsFaceTypeMax];
  double stdev[statsFaceTypeMax];

  //std::cout << "faceinfo.size: " << faceinfos.size() << std::endl;
  m_faceStats[statsFaceNum].push_back(faceinfos.size());
  if (faceinfos.size() != 1) {
    m_faceStats[statsFaceYaw].push_back(0.0f);
    m_faceStats[statsFacePitch].push_back(0.0f);
    m_faceStats[statsFaceRow].push_back(0.0f);
    m_faceStats[statsFaceArea].push_back(0.0f);
  } else {
    m_faceStats[statsFaceYaw].push_back(faceinfos[0].poses[0]);
    m_faceStats[statsFacePitch].push_back(faceinfos[0].poses[1]);
    m_faceStats[statsFaceRow].push_back(faceinfos[0].poses[2]);
    m_faceStats[statsFaceArea].push_back(
      static_cast<float>((faceinfos[0].rect.right - faceinfos[0].rect.left) * (faceinfos[0].rect.bottom - faceinfos[0].rect.top)) / (640 * 480));
  }

  // 1.face number should be exactly 1
  if (m_faceStats[statsFaceNum].full()) {
    get_mean_stdev(m_faceStats[statsFaceNum].vector(), mean[statsFaceNum], stdev[statsFaceNum]);
    if (stdev[statsFaceNum] == FACE_NUMBER_STABLE_VAL) {
      if (mean[statsFaceNum] == 1.0f) {
        //cout << "Nice, only 1 face!!" << endl;
      } else if (mean[statsFaceNum] == 0.0f) {
        msg = "No face found!!";
        return 7;
      } else {
        //cout << "More than 1 face!!" << endl;
        msg = "More than 1 face found!!";
        return 8;
      }
    } else {
      //cout << "Face number not stable!!" << endl;
	  msg = "keep stable!!";
      return 9;
    }
  } else {
    msg = "keep stable!!";
    return 9;
  }

  // face distance
  if (m_faceStats[statsFaceArea].full()) {
    get_mean_stdev(m_faceStats[statsFaceArea].vector(), mean[statsFaceArea], stdev[statsFaceArea]);
    if (stdev[statsFaceArea] < FACE_AREA_STABLE_THRES) {
      if (mean[statsFaceArea] > FACE_AREA_LEGAL_THRES) {
        //cout << "Nice, distance is OK!!" <<  mean[statsFaceArea] << endl;
      } else {
        //cout << "Distance is NOT OK!! "<< endl;
		msg = "Distance is NOT OK!!";
        return 11;
      }
    } else {
      //cout << "keep stable!!" << endl;
      msg = "keep stable!!";
	  return 9;
    }
  }

  // 3.face pose
  if (m_faceStats[statsFaceYaw].full() &&
    m_faceStats[statsFacePitch].full() &&
    m_faceStats[statsFaceRow].full())
  {
    get_mean_stdev(m_faceStats[statsFaceYaw].vector(), mean[statsFaceYaw], stdev[statsFaceYaw]);
    get_mean_stdev(m_faceStats[statsFacePitch].vector(), mean[statsFacePitch],stdev[statsFacePitch]);
    get_mean_stdev(m_faceStats[statsFaceRow].vector(), mean[statsFaceRow], stdev[statsFaceRow]);
    if (stdev[statsFaceYaw] < FACE_POSE_STABLE_THRES &&
      stdev[statsFacePitch] < FACE_POSE_STABLE_THRES &&
      stdev[statsFaceRow] < FACE_POSE_STABLE_THRES)
    {
      if (abs(mean[statsFaceYaw]) < FACE_POSE_YAW_LEGAL_THRES &&
        abs(mean[statsFacePitch]) < FACE_POSE_PITCH_LEGAL_THRES &&
        abs(mean[statsFaceRow]) < FACE_POSE_ROW_LEGAL_THRES)
      {
        cout << "Nice, degree is OK!!" << endl;
		msg = "check Face Pose success!!";
        return 0;
      } else {
        //printf("mean (%f, %f) (%f,%f) (%f,%f) \n",
        //mean[1],FACE_POSE_YAW_LEGAL_THRES, mean[2],FACE_POSE_PITCH_LEGAL_THRES, mean[3],FACE_POSE_ROW_LEGAL_THRES);
        //cout << "Degree is NOT OK!!" << endl;
		msg = "Degree is NOT OK!!";
        return 10;
      }
    } else {
      //printf("stable pose (%f, %f) (%f,%f) (%f,%f) \n",
      //stdev[1],FACE_POSE_STABLE_THRES, stdev[2],FACE_POSE_STABLE_THRES, stdev[3],FACE_POSE_STABLE_THRES);
      //cout << "Degree is not stable!!" << endl;
	  msg = "keep stable!!";
      return 9;
    }
  }

  //can't reach here
  msg = "keep stable!!";
  return 9;
}

int FaceManager::addFaceIDCacheInfo(std::string & name, bool is_host)
{
  m_faceIdCached.is_host = is_host;
  m_faceIdCached.name = name;

  return true;
}

int FaceManager::addFaceFeatureCacheInfo(std::vector<EntryFaceInfo> & faceinfo)
{
  m_faceFeatsCached = faceinfo[0].feats;
  return true;
}

int FaceManager::cancelAddFace()
{
  m_faceFeatsCached.clear();
  m_faceIdCached.name = "";
  m_faceIdCached.is_host = false;
  return 0;
}

int FaceManager::confirmFace(std::string &name, bool is_host)
{
  std::string filename;

  std::cout << "confirm last face name: " << m_faceIdCached.name << " is_host: " << m_faceIdCached.is_host << std::endl;
  if(m_faceIdCached.name.compare(name) != 0 || is_host != m_faceIdCached.is_host)
  {
    std::cout << "confirmFace face name: " << name << "but cache name:" << m_faceIdCached.name << std::endl;
    return -1;
  }
  if(m_faceIdCached.name.compare(name) != 0 || is_host != m_faceIdCached.is_host)
  {
    std::cout << "confirmFace face name: " << name << "but cache name:" << m_faceIdCached.name << std::endl;
    return -1;
  }
  if(m_faceFeatsCached.empty())
  {
    std::cout << "Error:faceFeatsCached empty..." << std::endl;
    return -1;
  }
  m_mutex.lock();
  m_features[m_faceIdCached.name] = m_faceFeatsCached;
  m_hostMap[m_faceIdCached.name] = m_faceIdCached.is_host;
  m_mutex.unlock();

  updateFeaturesFile();
  /* clear face cache */
  m_faceFeatsCached.clear();
  m_faceIdCached.name = "";
  m_faceIdCached.is_host = false;
  //m_faceCacheSize = 0;

  return 0;
}

int FaceManager::updateFaceId(std::string & ori_name, std::string & new_name)
{
  if (m_features.find(ori_name) == m_features.end()) {
    std::cout << "Face name not fount " << ori_name << std::endl;
    return -1;
  }

  m_mutex.lock();
  m_features[new_name] = m_features[ori_name];
  m_features.erase(ori_name);
  m_hostMap[new_name] = m_hostMap[ori_name];
  m_hostMap.erase(ori_name);
  m_mutex.unlock();

  updateFeaturesFile();

  return 0;

}

int FaceManager::deleteFace(std::string & face_name)
{
  if (m_features.find(face_name) == m_features.end()) {
    std::cout << "Face name not found:" << face_name << std::endl;
    return 0;
  }

  m_mutex.lock();
  m_features.erase(face_name);
  m_hostMap.erase(face_name);
  m_mutex.unlock();

  updateFeaturesFile();

  return 0;
}


std::string FaceManager::getAllFaces()
{

  std::string all_face_info;
  std::string face_name;
  int is_host;
  std::map<std::string, std::vector<float>>::iterator feature_iter;

  for (feature_iter = m_features.begin();feature_iter != m_features.end();feature_iter++)
  {
    face_name = feature_iter->first;
    is_host = m_hostMap[face_name];

    all_face_info = all_face_info + "id=" + face_name + ",host=" + std::to_string(is_host) + ";";
  }

  return all_face_info;
}


bool FaceManager::findFace(const std::string & face_name)
{
  return m_features.find(face_name) != m_features.end();
}

bool FaceManager::isHost(const std::string & face_name)
{
  if (m_hostMap.find(face_name) != m_hostMap.end()) {
    return m_hostMap[face_name];
  }

  return false;
}

}  // namespace cyberdog_vision
