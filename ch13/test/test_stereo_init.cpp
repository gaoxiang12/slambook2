#include <gtest/gtest.h>

#include "myslam/common_include.h"
#include "myslam/frontend.h"
#include "myslam/visual_odometry.h"

TEST(MyslamTest, StereoInit) {
    auto vo = std::make_shared<myslam::VisualOdometry>(FLAGS_config_file);
    EXPECT_EQ(vo->Init(), true);

    vo->Step();
    EXPECT_EQ(vo->GetFrontendStatus(), FrontendStatus::TRACKING_GOOD);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}