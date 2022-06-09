#ifndef CYBERDOG_VISION__BODY_DETECTION_HPP_
#define CYBERDOG_VISION__BODY_DETECTION_HPP_

#include <string>
#include <memory>

#include "ContentMotionAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class BodyDetection
{
public:
    BodyDetection(const std::string &model_det, const std::string &model_cls);
    ~BodyDetection();

    int Detect(const cv::Mat &img, std::vector<HumanBodyInfo> &infos);

private:
    std::shared_ptr<ContentMotionAPI> body_ptr_;
    int gpu_id_;

};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__BODY_DETECTION_HPP_