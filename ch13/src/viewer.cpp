//
// Created by gaoxiang on 19-5-4.
//
#include <pangolin/pangolin.h>

#include "myslam/viewer.h"

namespace myslam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::ThreadLoop() {

}

}
