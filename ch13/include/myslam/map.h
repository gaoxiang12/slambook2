#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {
class Map {
public:
    typedef std::shared_ptr<Map> Ptr;
    std::unordered_map<unsigned long, MapPoint::Ptr> landmarks_;        // all landmarks
    std::unordered_map<unsigned long, Frame::Ptr> keyframes_;           // all key-frames

    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);
};
}

#endif // MAP_H
