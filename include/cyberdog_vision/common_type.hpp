#ifndef CYBERDOG_VISION__COMMON_TYPE_HPP_
#define CYBERDOG_VISION__COMMON_TYPE_HPP_

#include <string>

#include <opencv2/opencv.hpp>

#include "XMFaceAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

struct SingleBodyInfo
{
  std::string id;
  float score;
  cv::Rect rect;
  std::vector<float> feats;
};

struct InferBbox
{
    cv::Rect body_box;
    float score;
};

inline void img_convert(const cv::Mat &img, XMImage &xm_img)
{
    xm_img.data = img.data;
    xm_img.width = img.cols;
    xm_img.height = img.rows;
    xm_img.channel = img.channels();
    xm_img.type = ColorType::BGR;
}

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__COMMON_TYPE_HPP_