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

#ifndef WAYFINDER_H
#define WAYFINDER_H

#include <string>

#include "server/azure_blob_loader.h"
#include "server/backbone_graph.h"

using namespace colmap;

const std::string kGraphFileName = "graph.json";

class Wayfinder
{
public:
    Wayfinder(const std::string& venue_name, const std::string& area_name,
              const std::shared_ptr<AzureBlobLoader> azure_blob_loader);
private:
    void FetchGraph();
    
    const std::string venue_name_;
    const std::string area_name_;
    
    BackboneGraph graph_;
    
    std::shared_ptr<AzureBlobLoader> azure_blob_loader_;
};

#endif // WAYFINDER_H
