#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/common_include.h"

namespace myslam {

class Dataset {
   public:
    Dataset(const std::string& dataset_path);
    bool Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

   private:
    std::string dataset_path_;
    int current_image_index_ = 0;
}

}  // namespace myslam

#endif