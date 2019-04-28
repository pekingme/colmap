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

#include "wayfinder.h"

Wayfinder::Wayfinder ( const std::string& venue_name, const std::string& area_name,
                       const std::shared_ptr<AzureBlobLoader> azure_blob_loader )
    :venue_name_ ( EnsureTrailingSlash ( venue_name ) ), area_name_ ( EnsureTrailingSlash ( area_name ) ),
     azure_blob_loader_ ( azure_blob_loader )
{
    std::cout << "Initializing wayfinder for " << venue_name_ << area_name_ << std::endl;

    // load graph json file.
    PrintHeading2 ( "Downloading graph" );
    std::string graph_name = venue_name_+area_name_+kGraphFileName;
    std::future<void> download_future = std::async ( std::launch::async, &AzureBlobLoader::LoadAreaGraph,
                                        azure_blob_loader_, graph_name, graph_name );
    download_future.get();
    std::cout << "  Done." << std::endl;

    // fetch graph from json file.
    std::ifstream graph_json_file ( graph_name );
    std::string graph_json ( ( std::istreambuf_iterator<char> ( graph_json_file ) ),
                             ( std::istreambuf_iterator<char>() ) );
    graph_ = BackboneGraph ( graph_json );
}
