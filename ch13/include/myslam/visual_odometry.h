#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/frontend.h"
#include "myslam/backend.h"
#include "myslam/viewer.h"

namespace myslam {

class VisualOdometry {
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;
    /// constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_;
    Backend::Ptr backend_;
    Map::Ptr map_;
    Viewer::Ptr viewer_;

};
}

#endif // MYSLAM_VISUAL_ODOMETRY_H
