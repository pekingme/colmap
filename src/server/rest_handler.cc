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

#include "rest_handler.h"
#include "pistache/http_headers.h"
#include "util/misc.h"

#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>

static void FetchFields ( const std::string& resource,
                          std::unordered_map<std::string, std::string>* field_map )
{
    std::vector<std::string> tokens;
    boost::split ( tokens, resource, boost::is_any_of ( "/" ) );
    for ( size_t i=0; i+1<tokens.size(); i+=2 ) {
        ( *field_map ) [tokens[i]] = tokens[i+1];
    }
}

RestHandler::RestHandler()
{
    azure_blob_loader_ = make_shared<AzureBlobLoader>();
}

void RestHandler::onRequest ( const Http::Request& request, Http::ResponseWriter response )
{
    PrintHeading2 ( "Request received" );
    std::cout << request.resource() << std::endl;

    PrintHeading2 ( "Parse parameters" );
    std::unordered_map<std::string, std::string> field_map;
    FetchFields ( request.resource(), &field_map );
    for ( std::pair<std::string, std::string> value : field_map ) {
        std::cout << value.first << ": " << value.second << std::endl;
    }

    if ( request.method() == Http::Method::Get ) {
        if ( field_map["func"] == "PictureLocalization" ) {
            ProcessLocalization ( field_map, &response );
        } else if ( field_map["func"] == "Wayfinding" ) {
            ProcessWayfinding ( field_map, &response );
        }
    }
}

void RestHandler::onTimeout ( const Http::Request& request, Http::ResponseWriter response )
{
    UNUSED ( request );
    response.send ( Http::Code::Request_Timeout, "Timeout" )
    .then ( [=] ( ssize_t ) {}, PrintException() );
}

// url: /api/func/PictureLocalization/venue/***/area/***/frame/***/camera_model/***/camera_params/***/
void RestHandler::ProcessLocalization ( const std::unordered_map<std::string, std::string>& field_map,
                                        Http::ResponseWriter* response )
{
    if ( field_map.find ( "venue" ) == field_map.end()
            || field_map.find ( "area" ) == field_map.end()
            || field_map.find ( "frame" ) == field_map.end()
            || field_map.find ( "camera_model" ) == field_map.end()
            || field_map.find ( "camera_params" ) == field_map.end() ) {
        std::cout << "Bad request" << std::endl;
        response->send ( Http::Code::Bad_Request, "Bad request" )
        .then ( [=] ( ssize_t ) {}, PrintException() );
        return;
    }

    const std::string venue_name = field_map.at ( "venue" );
    const std::string area_name = field_map.at ( "area" );
    const std::string image_csv_names = field_map.at ( "frame" );
    const std::string camera_model_name = field_map.at ( "camera_model" );
    const std::string camera_params_csv = field_map.at ( "camera_params" );

    // Split image names.
    std::vector<std::string> image_names;
    boost::split ( image_names, image_csv_names, boost::is_any_of ( "," ) );
    if ( image_names.empty() ) {
        response->send ( Http::Code::Ok, "No image to process" );
        return;
    }

    // Create localizer is not existed.
    if ( localizers_.find ( venue_name ) == localizers_.end() ) {
        localizers_[venue_name] = unordered_map<std::string, shared_ptr<Localizer>>();
    }
    if ( localizers_.at ( venue_name ).find ( area_name ) == localizers_.at ( venue_name ).end() ) {
        localizers_[venue_name][area_name] = make_shared<Localizer> ( venue_name, area_name, azure_blob_loader_ );
    }

    // Localize images.
    std::shared_ptr<Localizer> localizer = localizers_.at ( venue_name ).at ( area_name );
    std::future<std::pair<int, std::string>> future = std::async ( std::launch::async,
                                          &Localizer::Localize, localizer, camera_model_name,
                                          camera_params_csv, image_names );

    std::chrono::seconds process_duration ( 30 );

    if ( future.wait_for ( process_duration ) == std::future_status::timeout ) {
        response->send ( Http::Code::Ok, "Server timeout (30 seconds)" );
    } else {
        std::pair<int, std::string> result = future.get();
        if ( result.first == EXIT_FAILURE ) {
            response->send ( Http::Code::Internal_Server_Error, "Cannot localized" );
        } else {
            response->send ( Http::Code::Ok, result.second, MIME ( Application, Json ) );
        }
    }
}

// url: /api/func/Wayfinding/venue/***/area/***/location/***/destination/***/
void RestHandler::ProcessWayfinding ( const std::unordered_map<std::string, std::string>& field_map,
                                      Http::ResponseWriter* response )
{
    if ( field_map.find ( "venue" ) == field_map.end()
            || field_map.find ( "area" ) ==field_map.end()
            || field_map.find ( "location" ) == field_map.end()
            || field_map.find ( "destination" ) == field_map.end() ) {
        std::cout << "Bad request" << std::endl;
        response->send ( Http::Code::Bad_Request, "Bad request" )
        .then ( [=] ( ssize_t ) {}, PrintException() );
        return;
    }

    const std::string venue_name = field_map.at ( "venue" );
    const std::string area_name = field_map.at ( "area" );
    const std::vector<float> location = CSVToVector<float> ( field_map.at ( "location" ) );
    const long destination = std::stol ( field_map.at ( "destination" ) );

    // Create wayfinder if not existed.
    if ( wayfinders_.find ( venue_name ) ==wayfinders_.end() ) {
        wayfinders_[venue_name] = unordered_map<std::string, shared_ptr<Wayfinder>>();
    }
    if ( wayfinders_.at ( venue_name ).find ( area_name ) == wayfinders_.at ( venue_name ).end() ) {
        wayfinders_[venue_name][area_name] = make_shared<Wayfinder> ( venue_name, area_name, azure_blob_loader_ );
    }

    // Calculating path
    std::shared_ptr<Wayfinder> wayfinder = wayfinders_.at(venue_name).at(area_name);
    
    
    response->send(Http::Code::Internal_Server_Error, "Not implemented");
}
