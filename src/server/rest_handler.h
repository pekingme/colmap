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

#ifndef RESTHANDLER_H
#define RESTHANDLER_H

#include <unordered_map>
#include <string>

#include "pistache/http.h"
#include "server/azure_blob_loader.h"
#include "server/localizer.h"
#include "server/wayfinder.h"

using namespace Pistache;
using namespace colmap;

class RestHandler : public Http::Handler
{
public:
    RestHandler();

    HTTP_PROTOTYPE ( RestHandler );
    void onRequest ( const Http::Request & request, Http::ResponseWriter response ) override;
    void onTimeout ( const Http::Request & request, Http::ResponseWriter response ) override;

private:
    // Handle picture localization service request.
    void ProcessLocalization ( const std::unordered_map<std::string, std::string>& field_map,
                               Http::ResponseWriter* response );
    // Handle wayfinding request.
    void ProcessWayfinding ( const std::unordered_map<std::string, std::string>& field_map,
                             Http::ResponseWriter* response );

    // venue/area/localizer
    std::unordered_map<std::string, std::unordered_map<std::string, std::shared_ptr<Localizer>>> localizers_;
    // venue/area/wayfinder
    std::unordered_map<std::string, std::unordered_map<std::string, std::shared_ptr<Wayfinder>>> wayfinders_;
    
    std::shared_ptr<AzureBlobLoader> azure_blob_loader_;
};

#endif // RESTHANDLER_H
