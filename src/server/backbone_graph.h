/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2019  <copyright holder> <email>
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
 */

#ifndef BACKBONEGRAPH_H
#define BACKBONEGRAPH_H

#include <Eigen/Core>
#include <vector>
#include <unordered_map>

#include "rapidjson/document.h"

using namespace rapidjson;

class Waypoint;
class Landmark;

class Landmark
{
public:
    Landmark() = default;
    Landmark ( const Value& json_value );
    long id_;
private:
    std::string name_;
    std::string description_;
    std::string category_;
    bool cross_linked_;
    bool enable_in_trial_;
    Eigen::Vector3f position_;

    std::vector<long> waypoints_id_;
};

class Waypoint
{
public:
    Waypoint() = default;
    Waypoint ( const Value& json_value );
    long id_;
private:
    std::string name_;
    std::string description_;
    Eigen::Vector3f position_;

    std::vector<long> landmarks_id_;
    std::vector<long> waypoints_id_;
};

class MapRegion
{
public:
    MapRegion ( const Value& json_value );
private:
    long id_;
    std::string name_;
    std::vector<std::pair<int, int>> tiles_;
    std::vector<long> landmarks_id_;
    std::vector<long> waypoints_id_;
};

class AnnouncedRegion
{
public:
    AnnouncedRegion ( const Value& json_value );
private:
    long id_;
    std::string name_;
    std::vector<std::pair<int, int>> tiles_;
    bool is_corridor_;
    Eigen::Vector3f end1_;
    Eigen::Vector3f end2_;
};

class Area
{
public:
    Area ( const Value& json_value );
private:
    long id_;
    std::string name_;
    bool is_corridor_;
    Eigen::Vector3f corridor_end1_;
    Eigen::Vector3f corridor_end2_;
    std::vector<std::pair<int, int>> tiles_;

    std::vector<long> landmarks_id_;
    std::vector<long> waypoints_id_;
};

class BackboneGraph
{
public:
    BackboneGraph() = default;
    BackboneGraph ( const std::string& json );

    std::string ServeJsonString()
    {
        return json_string_;
    }

    float scale_;
private:
    std::unordered_map<long, Landmark> landmarks_;
    std::unordered_map<long, Waypoint> waypoints_;
    std::vector<MapRegion> map_regions_;
    std::vector<AnnouncedRegion> announced_regions_;
    //std::vector<Area> areas_;
    Eigen::Vector3f north_vector_;
    float tile_size_;
    std::string area_name_;

    std::string json_string_;
};

#endif // BACKBONEGRAPH_H
