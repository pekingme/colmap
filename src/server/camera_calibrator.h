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

#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include "server/azure_blob_loader.h"
#include "server/constants.h"
#include "util/option_manager.h"
#include "base/database_cache.h"
#include "controllers/incremental_mapper.h"

using namespace colmap;

class CameraCalibrator
{
public:
    CameraCalibrator(const std::shared_ptr<AzureBlobLoader> azure_blob_loader);
    
    void HandoverRequestProcess(const std::string& user_name,
        const std::vector<std::string>& request_image_names,
        std::function<void(const int, const std::string&)> complete_callback);
    
private:
    void InitializeParentFolder();
    void InitializeUser(const std::string user_id, OptionManager* options);
    
    void ExtractFeatures(OptionManager* options);
    void MatchImages(OptionManager* options);
    void RegisterImages(OptionManager* options, 
                        ReconstructionManager* reconstruction_manager);
    
    std::shared_ptr<AzureBlobLoader> azure_blob_loader_;
};

#endif // CAMERACALIBRATOR_H
