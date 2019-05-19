//
// Created by gaoxiang on 19-5-2.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"

namespace myslam {
class Map;

class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;
    Backend() {}

    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

   private:
    std::shared_ptr<Map> map_;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H