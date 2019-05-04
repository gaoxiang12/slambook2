#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Backend;

class Frontend {
public:
    typedef std::shared_ptr<Frontend> Ptr;
    enum class FrontendStatus {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    Frontend();

    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) { map_ = map; }

    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    FrontendStatus GetStatus() const { return status_; }

private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * estimate current frame's pose
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Frame::Ptr ref_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;

    // params
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;    // feature detector in opencv
};

}

#endif //MYSLAM_FRONTEND_H
