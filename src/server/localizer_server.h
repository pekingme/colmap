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

#ifndef LOCALIZER_SERVER_H_
#define LOCALIZER_SERVER_H_

#include <iostream>
#include <string>
#include <unordered_map>

#include "server/cpprest_import.h"
#include "server/localizer.h"

using namespace std;
using namespace colmap;
using namespace utility;
using namespace web;
using namespace http;
using namespace web::http::experimental::listener;

class LocalizerServer
{
public:
    enum ServiceType {
        // GET /api/LandmarkQuery/venue/{venue_name}/
        LANDMARK_QUERY,
        // GET /api/PictureLocalization/venue/{venue_name}/frame/{frame_csv_names}/
        PICTURE_LOCALIZATION,
        UNSUPPORTED,
    };

    LocalizerServer() {}
    LocalizerServer(const utility::string_t& url);

    pplx::task<void> open() {
        return listener_.open();
    }
    pplx::task<void> close() {
        return listener_.close();
    }

private:
    void HandleGet(http_request message);
    void HandlePut(http_request message);
    void HandlePost(http_request message);
    void HandleDelete(http_request message);

    // Determines which service the client is requesting.
    ServiceType FindServiceType(const http_request& message);

    // Handle landmark query service request.
    void ProcessLandmarkQuery(http_request* message);
    // Handle picture localization service request.
    void ProcessPictureLocalization(http_request* message);

    http_listener listener_;

    int next_request_id_ = 1;
    unordered_map<string, shared_ptr<Localizer>> requests_handlers_;
};

#endif // LOCALIZER_SERVER_H_

