add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
       # backend.cpp
        ../include/myslam/algorithm.h ../include/myslam/g2o_types.h)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
