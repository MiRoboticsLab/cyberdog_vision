#ifndef CYBERDOG_VISION__PERSON_REID_HPP_
#define CYBERDOG_VISION__PERSON_REID_HPP_

#include "ReIDToolAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

enum SimType {
    kSimOne2One = 0,
    kSimOne2Group,
    kSimGroup2Group
};

class PersonReID
{
public:
    PersonReID(const std::string &model_reid);
    ~PersonReID();

    int SetTracker(const cv::Mat &img, const cv::Rect &body_box, std::vector<float> &reid_feat);
    int GetReIDInfo(const cv::Mat &img, const std::vector<cv::Rect> &body_bboxes, int &id);
    int GetFeatureLen();
    void ResetTracker();

private:
    int GetFeature(const cv::Mat &img, const cv::Rect &body_box, std::vector<float> &reid_feat);
    float GetSim(std::vector<float> &feat_det, std::vector<float> &feat_library, const SimType &sim_type);
    
    int gpu_id_;
    int tracking_id_;
    int object_loss_th_;
    int library_frame_num_;
    int unmatch_count_;
    float feat_sim_th_;
    float feat_update_th_;

    void *reid_ptr_;
    std::vector<float> tracker_feat_;

};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__PERSON_REID_HPP_