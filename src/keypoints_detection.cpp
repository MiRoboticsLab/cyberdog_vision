#include "cyberdog_vision/keypoints_detection.hpp"


namespace cyberdog_vision
{

KeypointsDetection::KeypointsDetection(const std::string &model_keypoints)
{
    keypoints_ptr_ = std::make_shared<Person_keyPoints>(model_keypoints);
}

void KeypointsDetection::GetKeypointsInfo(const cv::Mat &img, const std::vector<InferBbox> &body_boxes, std::vector<std::vector<cv::Point2f> > &bodies_keypoints)
{
    XMImage xm_img;
    img_convert(img, xm_img);
    std::vector<pbox> infer_bboxes;
    for (auto &infer_bbox: body_boxes) {
        XMPoint point_tl = XMPoint(infer_bbox.body_box.x, infer_bbox.body_box.y);
        XMPoint point_br = XMPoint(infer_bbox.body_box.x + infer_bbox.body_box.width, infer_bbox.body_box.y + infer_bbox.body_box.height);
        infer_bboxes.push_back({point_tl, point_br});
    }
    bool is_save_keypoints = false;
    bool is_show_names = false;
    keypoints_ptr_ -> Inference(xm_img, infer_bboxes, is_save_keypoints, is_show_names);
    std::vector<std::vector<XMPoint> > xm_points = keypoints_ptr_ -> Get_Persons_Keypoints();
    for (size_t i = 0; i < xm_points.size(); ++i) {
        std::vector<cv::Point2f> single_body_keypoints;
        for (size_t j = 0; j < xm_points[i].size(); ++j) {
            single_body_keypoints.push_back(cv::Point2f(xm_points[i][j].x, xm_points[i][j].y));
        }
        bodies_keypoints.push_back(single_body_keypoints);
    }
}

KeypointsDetection::~KeypointsDetection()
{}

}  // namespace cyberdog_vision