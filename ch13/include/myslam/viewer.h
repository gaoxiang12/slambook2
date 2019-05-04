//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

private:
    void ThreadLoop();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

};
}

#endif //MYSLAM_VIEWER_H
