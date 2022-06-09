#include "cyberdog_vision/gesture_recognition.hpp"


namespace cyberdog_vision
{
GestureRecognition::GestureRecognition(const std::string &model_det,const std::string &model_cls)
{
    gesture_ptr_ = std::make_shared<Hand_Gesture_Classification_Node>(model_det, model_cls);
}

int GestureRecognition::GetGestureInfo(const cv::Mat &img, const std::vector<InferBbox> &body_boxes, int &gesture_cls, cv::Rect &gesture_box)
{
    std::vector<bbox> infer_bboxes;
    for (auto &body : body_boxes) {
        bbox infer_bbox;
        infer_bbox.xmin = body.body_box.x;
        infer_bbox.ymin = body.body_box.y;
        infer_bbox.xmax = body.body_box.x + body.body_box.width;
        infer_bbox.ymax = body.body_box.y + body.body_box.height;
        infer_bbox.score = body.score;
        infer_bboxes.push_back(infer_bbox);
    }
    XMImage xm_img;
    img_convert(img, xm_img);
    gesture_ptr_->Inference(xm_img, infer_bboxes);
    gesture_cls = gesture_ptr_->GetHandClassifacation();
    if (gesture_cls == GestureLabel::kInvalid) {
        return -1;
    }
    std::vector<float> position = gesture_ptr_->GetHandPositions();
    gesture_box = cv::Rect(position[0], position[1], position[2] - position[0], position[3] - position[1]);
    return 0;
}

GestureRecognition::~GestureRecognition()
{}

}  // namespace cyberdog_vision