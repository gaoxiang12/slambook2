/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

namespace myslam {

struct Frame;

struct Feature;

struct MapPoint {
public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0; // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();       // Position in world
    int observed_times_ = 0;    // being observed by feature matching algo.
    std::list<std::weak_ptr<Feature>> _observations;

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};
}

#endif // MYSLAM_MAPPOINT_H
