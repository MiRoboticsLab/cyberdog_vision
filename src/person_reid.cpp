#include "cyberdog_vision/person_reid.hpp"

namespace cyberdog_vision
{

PersonReID::PersonReID(const std::string &model_reid) : gpu_id_(0), tracking_id_(0), object_loss_th_(300), library_frame_num_(15), unmatch_count_(0), feat_sim_th_(0.8), feat_update_th_(0.9), reid_ptr_(nullptr)
{
    if (0 != REID_Init(reid_ptr_, model_reid.c_str(), gpu_id_)) {
        throw std::logic_error("Init person reid algo fial. ");
    }
}

int PersonReID::SetTracker(const cv::Mat &img, const cv::Rect &body_box, std::vector<float> &reid_feat)
{
    if (0 != GetFeature(img, body_box, reid_feat)) {
        std::cout << "GetFeature fail. " << std::endl;
        return -1;
    }
    tracker_feat_.assign(reid_feat.begin(), reid_feat.end());
    return 0;
}

int PersonReID::GetReIDInfo(const cv::Mat &img, const std::vector<cv::Rect> &body_bboxes, int &id)
{
    id = -1;
    for (size_t i = 0; i < body_bboxes.size(); ++i) {
        std::vector<float> feat;
        if (0 != GetFeature(img, body_bboxes[i], feat)) {
            return -1;
        }
        if (0 != tracker_feat_.size()) {
            double sim_val = GetSim(feat, tracker_feat_, SimType::kSimOne2Group);
            if (sim_val > feat_sim_th_) {
                // Match success
                unmatch_count_ = 0;
                id = tracking_id_;
                if (sim_val > feat_update_th_) {
                    // Update library feat
                    if ((int)tracker_feat_.size()/128 == library_frame_num_) {
                        tracker_feat_.erase(tracker_feat_.begin(), tracker_feat_.begin() + 128);
                    }
                    tracker_feat_.insert(tracker_feat_.end(), feat.begin(), feat.end());
                }
            } else {
                unmatch_count_++;
                if (unmatch_count_ > object_loss_th_) {
                    std::cout << "Object is lost. " << std::endl;
                    ResetTracker();
                }
            }
        } else {
            tracker_feat_.assign(feat.begin(), feat.end());
        }
    }
    return 0;
}

int PersonReID::GetFeatureLen()
{
    return REID_GetFeatLen();
}

void PersonReID::ResetTracker()
{
    tracker_feat_.clear();
    tracking_id_++;
}

int PersonReID::GetFeature(const cv::Mat &img, const cv::Rect &body_box, std::vector<float> &reid_feat)
{
    cv::Mat reid_img = img(body_box);
    XMReIDImage xm_reid_img;
    xm_reid_img.data = reid_img.data;
    xm_reid_img.height = reid_img.rows;
    xm_reid_img.width = reid_img.cols;
    xm_reid_img.fmt = XM_IMG_FMT_BGR;
    float *feat = nullptr;
    if (0 != REID_ExtractFeat(reid_ptr_, &xm_reid_img, feat)) {
        std::cout << "Extract reid feature fail." << std::endl;
        return -1;
    }
    if (feat != nullptr) {
        memcpy(reid_feat.data(), feat, sizeof(float) * REID_GetFeatLen());
        feat = nullptr;
    }
    return 0;
}

float PersonReID::GetSim(std::vector<float> &feat_det, std::vector<float> &feat_library, const SimType &sim_type)
{
    double sim_value;
    switch (sim_type) {
        case SimType::kSimOne2One:
            sim_value = REID_GetSimOfOne2One(reid_ptr_, feat_det.data(), feat_library.data());
            break;
        case SimType::kSimOne2Group:
            sim_value = REID_GetSimOfOne2Group(reid_ptr_, feat_det.data(), feat_library.data(), feat_library.size()/128);
            break;
        case SimType::kSimGroup2Group:
            sim_value = REID_GetSimOfGroup2Group(reid_ptr_, feat_det.data(), feat_det.size()/128, feat_library.data(),  feat_library.size()/128);
            break;
    }
    return sim_value;
}

PersonReID::~PersonReID()
{
  if (reid_ptr_) {
    REID_Release(reid_ptr_);
  }
}

}  // namespace cyberdog_vision