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
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/prettywriter.h>

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
    assert ( document.HasMember ( "mapRegions" ) );
    assert ( document.HasMember ( "announcedRegions" ) );
    assert ( document.HasMember ( "tileSize" ) );
    assert ( document.HasMember ( "northVec" ) );
    assert ( document.HasMember ( "scale" ) );
    assert ( document.HasMember ( "areaName" ) );

    assert ( document["landmarks"].IsArray() );
    assert ( document["waypoints"].IsArray() );
    assert ( document["mapRegions"].IsArray() );
    assert ( document["announcedRegions"].IsArray() );

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
    const Value& map_regions_v = document["mapRegions"];
    for ( SizeType i=0; i<map_regions_v.Size(); i++ ) {
        map_regions_.emplace_back ( map_regions_v[i] );
    }
    const Value& announced_regions_v = document["announcedRegions"];
    for ( SizeType i=0; i<announced_regions_v.Size(); i++ ) {
        announced_regions_.emplace_back ( announced_regions_v[i] );
    }

    tile_size_ = document["tileSize"].GetFloat();
    scale_ = document["scale"].GetFloat();
    area_name_ = document["areaName"].GetString();
    north_vector_ = JsonValueToEigenVector3f ( document["northVec"] );

    StringBuffer buffer;
    PrettyWriter<StringBuffer> writer ( buffer );
    document.Accept ( writer );

    json_string_ = buffer.GetString();
}

Landmark::Landmark ( const Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "description" ) );
    assert ( json_value.HasMember ( "category" ) );
    assert ( json_value.HasMember ( "crossLinked" ) );
    assert ( json_value.HasMember ( "enableInTrial" ) );
    assert ( json_value.HasMember ( "position" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    description_ = json_value["description"].GetString();
    category_ = json_value["category"].GetString();
    cross_linked_ = json_value["crossLinked"].GetBool();
    enable_in_trial_ = json_value["enableInTrial"].GetBool();
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
    assert ( json_value.HasMember ( "description" ) );
    assert ( json_value.HasMember ( "position" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );
    assert ( json_value.HasMember ( "landmarksId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    description_ = json_value["description"].GetString();
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

MapRegion::MapRegion ( const rapidjson::Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "tilesList" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );
    assert ( json_value.HasMember ( "landmarksId" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();

    assert ( json_value["tilesList"].IsArray() );
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
    const Value& tile_list_v = json_value["tilesList"];
    for ( SizeType i=0; i<tile_list_v.Size(); i++ ) {
        tiles_.emplace_back ( std::make_pair ( tile_list_v[i]["indexX"].GetInt(),
                                               tile_list_v[i]["indexZ"].GetInt() ) );
    }
}

AnnouncedRegion::AnnouncedRegion ( const rapidjson::Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "tilesList" ) );
    assert ( json_value.HasMember ( "isCorridor" ) );
    assert ( json_value.HasMember ( "end1" ) );
    assert ( json_value.HasMember ( "end2" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    is_corridor_ = json_value["isCorridor"].GetBool();
    end1_ = JsonValueToEigenVector3f ( json_value["end1"] );
    end2_ = JsonValueToEigenVector3f ( json_value["end2"] );

    assert ( json_value["tilesList"].IsArray() );

    const Value& tile_list_v = json_value["tilesList"];
    for ( SizeType i=0; i<tile_list_v.Size(); i++ ) {
        tiles_.emplace_back ( std::make_pair ( tile_list_v[i]["indexX"].GetInt(),
                                               tile_list_v[i]["indexZ"].GetInt() ) );
    }
}
/*
Area::Area ( const rapidjson::Value& json_value )
{
    assert ( json_value.HasMember ( "id" ) );
    assert ( json_value.HasMember ( "name" ) );
    assert ( json_value.HasMember ( "tilesList" ) );
    assert ( json_value.HasMember ( "waypointsId" ) );
    assert ( json_value.HasMember ( "landmarksId" ) );
    assert ( json_value.HasMember ( "isCorridor" ) );
    assert ( json_value.HasMember ( "corridorEnd1" ) );
    assert ( json_value.HasMember ( "corridorEnd2" ) );

    id_ = json_value["id"].GetInt64();
    name_ = json_value["name"].GetString();
    is_corridor_ = json_value["isCorridor"].GetBool();
    corridor_end1_ = JsonValueToEigenVector3f ( json_value["corridorEnd1"] );
    corridor_end2_ = JsonValueToEigenVector3f ( json_value["corridorEnd2"] );

    assert ( json_value["tilesList"].IsArray() );
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
    const Value& tile_list_v = json_value["tilesList"];
    for ( SizeType i=0; i<tile_list_v.Size(); i++ ) {
        tiles_.emplace_back ( std::make_pair ( tile_list_v[i]["indexX"].GetInt(),
                                               tile_list_v[i]["indexZ"].GetInt() ) );
    }
}*/
