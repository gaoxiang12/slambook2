#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

namespace myslam {

struct Frame;

struct Feature;

struct MapPoint {
public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0; // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();       // Position in world
    std::mutex pos_mutex;
    int observed_times_ = 0;    // being observed by feature matching algo.
    std::list<std::weak_ptr<Feature>> _observations;

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(pos_mutex);
        return pos_;
    }

    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(pos_mutex);
        pos_ = pos;
    };

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};
}

#endif // MYSLAM_MAPPOINT_H
