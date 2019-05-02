#include "myslam/config.h"

namespace myslam {
void Config::setParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        LOG(WARNING) << "parameter file " << filename << " does not exist.";
        config_->file_.release();
        return;
    }
}

Config::~Config() {
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}
