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
#include <QApplication>

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
        } else if ( field_map["func"] == "FetchGraph" ) {
            ServeGraph ( field_map, &response );
        } else if ( field_map["func"]=="CalibrateCamera" ) {
            CalibrateCamera ( field_map, &response );
        } else if ( field_map["func"]=="QueryAreas" ) {
            QueryAreas ( field_map, &response );
        } else {
            TestMemory ( &response );
        }
    }
}

void RestHandler::onTimeout ( const Http::Request& request, Http::ResponseWriter response )
{
    UNUSED ( request );
    response.send ( Http::Code::Request_Timeout, "Timeout" )
    .then ( [=] ( ssize_t ) {}, PrintException() );
}

void TestMemoryInner ( std::function<void() > complete_callback )
{
    std::vector<long> junk;
    for ( int i=0; i<10000000; i++ ) {
        junk.emplace_back ( i );
    }
    complete_callback();
}

void RestHandler::TestMemory ( Http::ResponseWriter* response )
{
    std::async ( std::launch::async, TestMemoryInner, [response] {
        response->send ( Http::Code::Ok, "Done" );
    } );
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

    // Create localizer if not existed.
    if ( localizers_.find ( venue_name ) == localizers_.end() ) {
        localizers_[venue_name] = unordered_map<std::string, shared_ptr<Localizer>>();
    }
    if ( localizers_.at ( venue_name ).find ( area_name ) == localizers_.at ( venue_name ).end() ) {
        localizers_[venue_name][area_name] = make_shared<Localizer> ( venue_name, area_name, azure_blob_loader_ );
    }

    for ( auto& pair : localizers_ ) {
        std::cout << pair.first << std::endl;
        for ( auto& pair2 : pair.second ) {
            std::cout << pair2.first << std::endl;
        }
    }

    // Localize images.
    std::shared_ptr<Localizer> localizer = localizers_.at ( venue_name ).at ( area_name );
    localizer->HandoverRequestProcess ( area_name, camera_model_name, camera_params_csv,image_names,
    [response] ( const int result_code, const string& response_content ) {
        std::cout << result_code << ": " << response_content << std::endl;
        if ( result_code == EXIT_FAILURE ) {
            //response->send(Http::Code::Internal_Server_Error, response_content);
            response->timeout();
        } else {
            response->send ( Http::Code::Ok, response_content );
        }
    } );
}

// url: /api/func/FetchGraph/venue/***/area/***/
void RestHandler::ServeGraph ( const std::unordered_map<std::string, std::string>& field_map,
                               Http::ResponseWriter* response )
{
    if ( field_map.find ( "venue" ) == field_map.end()
            || field_map.find ( "area" ) ==field_map.end() ) {
        std::cout << "Bad request" << std::endl;
        response->send ( Http::Code::Bad_Request, "Bad request" )
        .then ( [=] ( ssize_t ) {}, PrintException() );
        return;
    }

    const std::string venue_name = field_map.at ( "venue" );
    const std::string area_name = field_map.at ( "area" );

    // Create wayfinder if not existed.
    if ( wayfinders_.find ( venue_name ) ==wayfinders_.end() ) {
        wayfinders_[venue_name] = unordered_map<std::string, shared_ptr<Wayfinder>>();
    }
    if ( wayfinders_.at ( venue_name ).find ( area_name ) == wayfinders_.at ( venue_name ).end() ) {
        wayfinders_[venue_name][area_name] = make_shared<Wayfinder> ( venue_name, area_name, azure_blob_loader_ );
    }

    // Serve graph json.
    std::shared_ptr<Wayfinder> wayfinder = wayfinders_.at ( venue_name ).at ( area_name );
    wayfinder->HandoverRequestProcess ( [response] ( const string& response_content ) {
        //std::cout << response_content << std::endl;
        response->send ( Http::Code::Ok, response_content );
    } );
}

// url: /api/func/CalibrateCamera/frame/***/
void RestHandler::CalibrateCamera ( const std::unordered_map<std::string, std::string>& field_map,
                                    Http::ResponseWriter* response )
{
    if ( field_map.find ( "frame" ) == field_map.end() ) {
        std::cout << "Bad request" << std::endl;
        response->send ( Http::Code::Bad_Request, "Bad request" )
        .then ( [=] ( ssize_t ) {}, PrintException() );
        return;
    }

    const std::string image_csv_names = field_map.at ( "frame" );
    const std::string user_name = GenerateRandomString ( 10 );

    // Split image names.
    std::vector<std::string> image_names;
    boost::split ( image_names, image_csv_names, boost::is_any_of ( "," ) );
    if ( image_names.empty() ) {
        response->send ( Http::Code::Ok, "No image to process" );
        return;
    }

    // Create camera calibrator instance if not exists.
    if ( !camera_calibrator_ ) {
        camera_calibrator_ = std::make_shared<CameraCalibrator> ( azure_blob_loader_ );
    }

    // Process camera calibration.
    camera_calibrator_->HandoverRequestProcess ( user_name, image_names,
    [response, user_name] ( const int result_code, const string& response_content ) {
        std::cout << "Camera parameters for " << user_name << std::endl;
        std::cout << result_code << ": " << response_content << std::endl;
        if ( result_code == EXIT_FAILURE ) {
            response->send ( Http::Code::Internal_Server_Error, response_content );
        } else {
            response->send ( Http::Code::Ok, response_content );
        }
    } );
}

// url: /api/func/QueryAreas/venue/***/
void RestHandler::QueryAreas ( const std::unordered_map<std::string, std::string>& field_map,
        Http::ResponseWriter* response )
{
    if ( field_map.find ( "venue" ) == field_map.end() ) {
        std::cout << "Bad request" << std::endl;
        response->send ( Http::Code::Bad_Request, "Bad request" )
        .then ( [=] ( ssize_t ) {}, PrintException() );
        return;
    }

    const std::string venue_name = field_map.at ( "venue" );
    std::vector<std::string> area_names = GetDirListNameOnly ( venue_name );
    std::string response_content = "";
    
    for ( std::string area_name : area_names ) {
        response_content += area_name+":";
    }
    
    response->send(Http::Code::Ok, response_content.substr(0, response_content.length()-1));
}
