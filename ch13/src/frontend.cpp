//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/features2d.hpp>
#include "myslam/frontend.h"
#include "myslam/config.h"
#include "myslam/feature.h"

namespace myslam {

Frontend::Frontend() {
    gftt_ = cv::GFTTDetector::create(
            Config::get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::get<int>("num_features_init");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case INITING:
            StereoInit();
            break;
        case TRACKING_GOOD:
            break;
        case TRACKING_BAD:
            break;
        case LOST:
            break;
    }
}

bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_)
        return false;

    return BuildInitMap();
}

int Frontend::DetectFeatures() {
    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints);
    for (auto &kp: keypoints) {
        current_frame_->features_left_.push_back(std::make_shared<Feature>(kp));
    }
    return keypoints.size();
}

int Frontend::FindFeaturesInRight() {

}

bool Frontend::BuildInitMap() {
    return true;
}

}