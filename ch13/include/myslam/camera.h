#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

// Pinhole RGBD camera model
class Camera {
public:
    typedef std::shared_ptr<Camera> Ptr;
    float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;  // Camera intrinsics

    Camera();

    Camera(float fx, float fy, float cx, float cy, float baseline) :
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline) {}

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

};

}
#endif // MYSLAM_CAMERA_H
