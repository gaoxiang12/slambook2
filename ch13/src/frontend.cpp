//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video.hpp>

#include "myslam/algorithm.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/map.h"
#include "myslam/g2o_types.h"

namespace myslam {

Frontend::Frontend() {
    gftt_ =
            cv::GFTTDetector::create(Config::get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::get<int>("num_features_init");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    int num_track_last = TrackLastFrame();
    EstimateCurrentPose();

    if (num_track_last > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (num_track_last > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    return true;
}

int Frontend::EstimateCurrentPose() {

}

int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {
        kps_last.push_back(kp->position_.pt);
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_,
                             kps_last, kps_current, status, error);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            current_frame_->features_left_.push_back(
                    std::make_shared<Feature>(current_frame_, kp));
            num_good_pts++;
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
}

bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_) return false;

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints);
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
                std::make_shared<Feature>(current_frame_, kp));
    }
    return keypoints.size();
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_,
                             current_frame_->right_img_, kps_left, kps_right,
                             status, error);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            current_frame_->features_right_.push_back(
                    std::make_shared<Feature>(current_frame_, kp));
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
}

bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
                camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();
        triangulation(poses, points, pworld);

        auto new_map_point = MapPoint::CreateNewMappoint();
        new_map_point->pos_ = pworld;
        new_map_point->observed_times_ = 2;
        new_map_point->_observations.push_back(
                current_frame_->features_left_[i]);
        new_map_point->_observations.push_back(
                current_frame_->features_right_[i]);

        map_->InsertMapPoint(new_map_point);
    }
    map_->InsertKeyFrame(current_frame_);

    return true;
}

}  // namespace myslam