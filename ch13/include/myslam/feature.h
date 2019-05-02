//
// Created by gaoxiang on 19-5-2.
//
#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>

namespace myslam {

struct Frame;
struct MapPoint;

struct Feature {
public:
    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_;
    bool is_outlier_ = false;

public:
    Feature() {}

    Feature(const cv::KeyPoint &kp) : position_(kp) {}
};
}

#endif //MYSLAM_FEATURE_H
