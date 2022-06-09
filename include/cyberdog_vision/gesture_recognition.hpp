#ifndef CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_
#define CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_

#include "person_detect_hand_gesture.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class GestureRecognition
{
public:
    GestureRecognition(const std::string &model_det,const std::string &model_cls);
    ~GestureRecognition();

    int GetGestureInfo(const cv::Mat &img, const std::vector<InferBbox> &body_boxes, int &gesture_cls, cv::Rect &gesture_box);

private:
    std::shared_ptr<Hand_Gesture_Classification_Node> gesture_ptr_;
};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__GESTURE_RECOGNITION_HPP_