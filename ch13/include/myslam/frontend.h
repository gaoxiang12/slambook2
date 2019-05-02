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
    enum FrontendStatus {
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
    FrontendStatus status_ = INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr ref_frame_ = nullptr;
    Camera::Ptr camera_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;

    // params
    int num_features_init_ = 100;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;    // feature detector in opencv
};

}

#endif //MYSLAM_FRONTEND_H
