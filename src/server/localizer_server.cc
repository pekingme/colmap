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

#include "server/localizer_server.h"
#include "util/misc.h"
#include "util/threading.h"

#include "boost/algorithm/string.hpp"

static bool FetchFields(const web::http::http_request& message,
                        const vector<std::__cxx11::string>& field_names,
                        vector<std::__cxx11::string>* field_values)
{
    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));

    if(paths.size() % 2 != 0) {
        std::cerr << "Number of components in url is not even" << std::endl;
        return false;
    }

    unordered_map<string, string> value_map;
    for(size_t i=0; i<paths.size(); i+=2) {
        value_map[paths[i]] = paths[i+1];
    }

    field_values->clear();
    for(size_t i=0; i<field_names.size(); i++) {
        if(value_map.count(field_names[i]) == 0) {
            std::cerr << "Position of components in url is not correct" << std::endl;
            return false;
        }
        field_values->emplace_back(value_map[field_names[i]]);
        std::cout << value_map[field_names[i]] << std::endl;
    }

    return true;
}

LocalizerServer::LocalizerServer(const utility::string_t& url) : listener_(url), thread_pool_(2)
{
    listener_.support(methods::GET, std::bind(&LocalizerServer::HandleGet, this, std::placeholders::_1));
    listener_.support(methods::PUT, std::bind(&LocalizerServer::HandlePut, this, std::placeholders::_1));
    listener_.support(methods::POST, std::bind(&LocalizerServer::HandlePost, this, std::placeholders::_1));
    listener_.support(methods::DEL, std::bind(&LocalizerServer::HandleDelete, this, std::placeholders::_1));

    // TODO Initialize request logger here.
}

LocalizerServer::ServiceType LocalizerServer::FindServiceType(const web::http::http_request& message)
{
    vector<string> field_values, field_names {"func"};
    if(!FetchFields(message, field_names, &field_values)) {
        return ServiceType::UNSUPPORTED;
    }

    string service_type_name = field_values[0];
    if( service_type_name == "LandmarkQuery") {
        return ServiceType::LANDMARK_QUERY;
    } else if(service_type_name == "PictureLocalization") {
        return ServiceType::PICTURE_LOCALIZATION;
    } else {
        return ServiceType::UNSUPPORTED;
    }
}

void LocalizerServer::ProcessLandmarkQuery(web::http::http_request* message)
{
    vector<string> field_values, field_names {"venue"};
    if(!FetchFields(*message, field_names, &field_values)) {
        message->reply(status_codes::BadRequest);
        return;
    }

    string venue_name = field_values[0];
    // TODO reply landmarks
}

// /api/func/{service_type}/venue/{venue_name}/camera_model/{camera_model_name}...
// /camera_params/{params_csv}/frame/{frame_names_csv}/
void LocalizerServer::ProcessPictureLocalization(web::http::http_request* message)
{
    vector<string> field_values, field_names {
        "venue", "frame", "camera_model", "camera_params"
    };
    if(!FetchFields(*message, field_names, &field_values)) {
        message->reply(status_codes::BadRequest);
        return;
    }

    string venue_name = field_values[0];
    string image_csv_names = field_values[1];
    string camera_model_name = field_values[2];
    string camera_params_csv = field_values[3];
    vector<string> image_names;
    boost::split(image_names, image_csv_names, [](char c) {
        return c == ',';
    });

    if(image_names.empty()) {
        message->reply(status_codes::OK, "No image to process");
        return;
    }

    thread_pool_.AddTask([message, venue_name, image_names,
    camera_model_name, camera_params_csv]() {
        Localizer localizer (venue_name, image_names);
        localizer.CalculateLocation(camera_model_name, camera_params_csv);
        if(localizer.CollectStatus() == EXIT_FAILURE) {
            message->reply(status_codes::InternalError);
        } else {
            message->reply(status_codes::OK, localizer.CollectResult());
        }
    });
    thread_pool_.Wait();
}

// /api/func/{service_type}/...
void LocalizerServer::HandleGet(web::http::http_request message)
{
    std::cout << message.relative_uri().to_string() << std::endl;
    ServiceType service_type = FindServiceType(message);

    switch(service_type) {
    case LANDMARK_QUERY:
        ProcessLandmarkQuery(&message);
        break;
    case PICTURE_LOCALIZATION:
        ProcessPictureLocalization(&message);
        break;
    case UNSUPPORTED:
        message.reply(status_codes::BadRequest);
        break;
    }
}

void LocalizerServer::HandlePut(web::http::http_request message)
{
}

void LocalizerServer::HandlePost(web::http::http_request message)
{
}

void LocalizerServer::HandleDelete(web::http::http_request message)
{
}
