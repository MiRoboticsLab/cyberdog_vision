#ifndef CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_
#define CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_

#include "person_keypoints.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class KeypointsDetection
{
public:
    KeypointsDetection(const std::string &model_keypoints);
    ~KeypointsDetection();

    void GetKeypointsInfo(const cv::Mat &img, const std::vector<InferBbox> &body_boxes, std::vector<std::vector<cv::Point2f> > &bodies_keypoints);

private:
    std::shared_ptr<Person_keyPoints> keypoints_ptr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__KEYPOINTS_DETECTION_HPP_