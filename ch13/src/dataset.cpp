#include "myslam/dataset.h"
#include "myslam/frame.h"

namespace myslam {

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

Frame::Ptr Dataset::NextFrame() {
    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_;
}

}  // namespace myslam