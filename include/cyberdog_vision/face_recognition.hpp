#ifndef CYBERDOG_VISION__FACE_RECOGNITION_HPP_
#define CYBERDOG_VISION__FACE_RECOGNITION_HPP_

#include <memory>

#include "XMFaceAPI.h"
#include "common_type.hpp"

namespace cyberdog_vision
{

class FaceRecognition
{
public:
    FaceRecognition(const std::string &model_det, const std::string &model_lmk, const std::string &model_feat, const std::string &model_emotion, bool open_emotion);    ~FaceRecognition();

    int GetFaceInfo(const cv::Mat &img, std::vector<EntryFaceInfo>& faces_info);
    int GetRecognitionResult(const cv::Mat &img, const std::map<std::string, std::vector<float> >& endlib_feats, std::vector<MatchFaceInfo>& faces_info);

private:
    XMFaceAPI* face_ptr_;

};

}  // namespace cyberdog_vision

#endif  // CYBERDOG_VISION__FACE_RECOGNITION_HPP_