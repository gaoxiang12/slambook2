//
// Created by gaoxiang on 19-5-4.
//

#include "myslam/visual_odometry.h"
#include "myslam/config.h"

namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path) : config_file_path_(config_path) {

}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    // create components and links
    frontend_ = std::make_shared<Frontend>();
    backend_ = std::make_shared<Backend>();
    map_ = std::make_shared<Map>();
    viewer_ = std::make_shared<Viewer>();

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {

}

}
