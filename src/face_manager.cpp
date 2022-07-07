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

#include "cyberdog_vision/face_manager.hpp"

namespace cyberdog_vision
{

static const char * label_path = "/home/mi/.faces/";

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

bool FaceManager::loadFeatures()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<std::string> filenames;
  if (access(label_path, 0) != 0) {
    std::cout << "faces path not found." << std::endl;
    return false;
  }
  cv::glob(std::string(label_path) + "*.data", filenames);

  for (unsigned int i = 0; i < filenames.size(); i++) {
    int face_name_len = filenames[i].find_last_of(".") - filenames[i].find_last_of("/") - 1;
    std::string face_name = filenames[i].substr(filenames[i].find_last_of("/") + 1, face_name_len);

    size_t feats_size;
    FILE * fp = fopen(filenames[i].c_str(), "rb");
    fread(&feats_size, sizeof(feats_size), 1, fp);
    if (feats_size == 0) {
      continue;
    }

    float * feats_data = new float[feats_size];
    bool is_host;
    fread(feats_data, sizeof(float), feats_size, fp);
    fread(&is_host, sizeof(is_host), 1, fp);

    std::vector<float> feats(feats_size);
    std::copy(feats_data, feats_data + feats_size, feats.begin());
    m_features[face_name] = feats;
    m_hostMap[face_name] = is_host;
    std::cout << "Load known face info " << face_name << "host:" << is_host << std::endl;
    fclose(fp);
    delete[] feats_data;
  }

  return true;
}

bool FaceManager::checkFacePose(std::vector<EntryFaceInfo> &faceinfo)
{
  std::cout << "faceinfo.size: " << faceinfo.size() << std::endl;
  if(faceinfo.size() == 1) return true;
  return false;
}

int FaceManager::addFaceIDCacheInfo(std::string & name,bool is_host)
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

int FaceManager::confirmFace(std::string &name,bool is_host)
{
  std::string filename;
  FILE * fp;

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

  /* save face features */
  if (access(label_path, 0) != 0) {
  umask(0);
  mkdir(label_path, 0755);
  }

  /*
   * face data layout
   * 1. features size   - unsigned long
   * 2. features        - float array
   * 3. host flag       - bool
   */
  size_t feats_size = m_faceFeatsCached.size();
  float * feats_array = new float[feats_size];
  std::copy(m_faceFeatsCached.begin(), m_faceFeatsCached.end(), feats_array);
  filename = std::string(label_path) + m_faceIdCached.name + ".data";
  fp = fopen(filename.c_str(), "wb+");
  fwrite(&feats_size, sizeof(feats_size), 1, fp);
  fwrite(feats_array, sizeof(float), feats_size, fp);
  fwrite(&m_faceIdCached.is_host, sizeof(m_faceIdCached.is_host), 1, fp);
  fclose(fp);
  delete[] feats_array;

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

  std::string ori_filename = std::string(label_path) + ori_name + ".data";
  std::string new_filename = std::string(label_path) + new_name + ".data";
  if (access(ori_filename.c_str(), 0) != 0) {
    printf("File %s not found", ori_filename.c_str());
    return -1;
  }

  if (rename(ori_filename.c_str(), new_filename.c_str()) != 0) {
    std::cout << "Rename file failed:" << ori_filename << std::endl;
    return -1;
  }

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

  /*del face file*/
  std::string filename = std::string(label_path) + face_name + ".data";
  if (access(filename.c_str(), 0) != 0) {
    std::cout << "File name not found:" << filename << std::endl;
    return 0;
  }

  if (unlink(filename.c_str()) != 0) {
    std::cout << "Remove file failed:" << filename << std::endl;
    return -1;
  }

  return 0;
}


std::string FaceManager::getAllFaces()
{
  std::vector<std::string> filenames;
  std::string res;
  std::string all_face_info;

  std::string path = getFaceDataPath();
  if (access(path.c_str(), 0) != 0) {
    return "NULL";
  }
  cv::glob(path + "*.data", filenames);

  for (unsigned int i = 0; i < filenames.size(); i++)
  {
    int face_name_len = filenames[i].find_last_of(".") - filenames[i].find_last_of("/") - 1;
    std::string face_name = filenames[i].substr(filenames[i].find_last_of("/") + 1, face_name_len);
    std::string is_host = "FALSE";
    if (FaceManager::getInstance()->isHost(face_name)) {
      is_host = "TRUE";
    }
    if(i != 0)
      all_face_info += ";";

    all_face_info = all_face_info + "id=" + face_name + ",host=" + is_host;

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
