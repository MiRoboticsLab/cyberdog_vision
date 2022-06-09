#include "cyberdog_vision/body_detection.hpp"

namespace cyberdog_vision
{

BodyDetection::BodyDetection(const std::string &model_det, const std::string &model_cls) : gpu_id_(0)
{
    body_ptr_ = std::make_shared<ContentMotionAPI>();
    if (0 != body_ptr_->Init(model_det, "", model_cls, gpu_id_)) {
        throw std::logic_error("Init body detection algo fial. ");
    }
}

int BodyDetection::Detect(const cv::Mat &img, std::vector<HumanBodyInfo> &infos)
{
    XMImage xm_img;
    img_convert(img, xm_img);
    struct LogInfo log_info;
    if (0 != body_ptr_->GetContentMotionAnalyse(xm_img, infos, log_info, gpu_id_)) {
        std::cout << "Detacte body fail. " << std::endl;
        return -1;
    }
    return 0;
}

BodyDetection::~BodyDetection()
{}

}  // namespace cyberdog_vision