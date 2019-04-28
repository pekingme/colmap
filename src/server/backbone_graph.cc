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

#include "backbone_graph.h"

Eigen::Vector3f JsonValueToEigenVector3f ( const Value& json )
{
    assert ( json.HasMember ( "x" ) );
    assert ( json.HasMember ( "y" ) );
    assert ( json.HasMember ( "z" ) );

    Eigen::Vector3f vec ( json["x"].GetFloat(), json["y"].GetFloat(), json["z"].GetFloat() );
    return vec;
}

BackboneGraph::BackboneGraph ( const std::string& json )
{
    Document document;
    document.Parse ( json.c_str() );

    assert ( document.HasMember ( "landmarks" ) );
    assert ( document.HasMember ( "waypoints" ) );
    assert ( document.HasMember ( "areas" ) );
    assert ( document.HasMember ( "tileSize" ) );
    assert ( document.HasMember ( "northVec" ) );
    assert ( document.HasMember ( "scale" ) );

    assert ( document["landmarks"].IsArray() );
    assert ( document["waypoints"].IsArray() );
    assert ( document["areas"].IsArray() );

    const Value& landmarks_v = document["landmarks"];
    for ( SizeType i=0; i<landmarks_v.Size(); i++ ) {
        Landmark landmark ( landmarks_v[i] );
        landmarks_[landmark.id_] = landmark;
    }
    const Value& waypoints_v = document["waypoints"];
    for ( SizeType i=0; i<waypoints_v.Size(); i++ ) {
        Waypoint waypoint ( waypoints_v[i] );
        waypoints_[waypoint.id_] = waypoint;
    }
    const Value& areas_v = document["areas"];
    for ( SizeType i=0; i<areas_v.Size(); i++ ) {
        areas_.emplace_back ( areas_v[i] );
    }
    tile_size_ = document["tileSize"].GetFloat();
    scale_ = document["scale"].GetFloat();
    north_vector_ = JsonValueToEigenVector3f ( document["northVec"] );
}

Landmark::Landmark ( const Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "description" ) );
    assert ( json_value.HasMember ( "position" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    description_ = json_value["description"].GetString();
    position_ = JsonValueToEigenVector3f ( json_value["position"] );

    assert ( json_value["waypointsId"].IsArray() );

    const Value& waypoints_id_v = json_value["waypointsId"];
    for ( SizeType i=0; i<waypoints_id_v.Size(); i++ ) {
        waypoints_id_.emplace_back ( waypoints_id_v[i].GetInt64() );
    }
}

Waypoint::Waypoint ( const rapidjson::Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "position" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );
    assert ( json_value.HasMember ( "landmarksId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    position_ = JsonValueToEigenVector3f ( json_value["position"] );

    assert ( json_value["waypointsId"].IsArray() );
    assert ( json_value["landmarksId"].IsArray() );

    const Value& waypoints_id_v = json_value["waypointsId"];
    for ( SizeType i=0; i<waypoints_id_v.Size(); i++ ) {
        waypoints_id_.emplace_back ( waypoints_id_v[i].GetInt64() );
    }
    const Value& landmarks_id_v = json_value["landmarksId"];
    for ( SizeType i=0; i<landmarks_id_v.Size(); i++ ) {
        landmarks_id_.emplace_back ( landmarks_id_v[i].GetInt64() );
    }
}

Area::Area ( const rapidjson::Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "tileXList" ) );
    assert ( json_value.HasMember ( "tileYList" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );
    assert ( json_value.HasMember ( "landmarksId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();

    assert ( json_value["tileXList"].IsArray() );
    assert ( json_value["tileYList"].IsArray() );
    assert ( json_value["waypointsId"].IsArray() );
    assert ( json_value["landmarksId"].IsArray() );

    const Value& waypoints_id_v = json_value["waypointsId"];
    for ( SizeType i=0; i<waypoints_id_v.Size(); i++ ) {
        waypoints_id_.emplace_back ( waypoints_id_v[i].GetInt64() );
    }
    const Value& landmarks_id_v = json_value["landmarksId"];
    for ( SizeType i=0; i<landmarks_id_v.Size(); i++ ) {
        landmarks_id_.emplace_back ( landmarks_id_v[i].GetInt64() );
    }
    const Value& tile_x_v = json_value["tileXList"];
    const Value& tile_y_v = json_value["tileYList"];
    assert ( tile_x_v.Size() == tile_y_v.Size() );
    for ( SizeType i=0; i<tile_x_v.Size(); i++ ) {
        tiles_.emplace_back(std::make_pair(tile_x_v[i].GetInt(), tile_y_v[i].GetInt()));
    }
}
