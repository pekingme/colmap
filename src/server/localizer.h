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

#ifndef LOCALIZER2_H
#define LOCALIZER2_H

#include <future>
#include <string>
#include <iostream>
#include <functional>

#include "server/azure_blob_loader.h"
#include "server/localization_result.h"
#include "server/landmark_info.h"
#include "server/constants.h"
#include "feature/extraction.h"
#include "retrieval/visual_index.h"
#include "base/database_cache.h"
#include "util/option_manager.h"
#include "util/threading.h"
#include "util/misc.h"

using namespace colmap;

class Localizer
{
public:
    Localizer ( const std::string& venue_name, const std::string& area_name,
                const std::shared_ptr<AzureBlobLoader> azure_blob_loader );

    // Should not block
    void HandoverRequestProcess (
        const std::string& area_name,
        const std::string& camera_model_name,
        const std::string& camera_params_csv,
        const std::vector<std::string>& request_image_names,
        std::function<void ( const int, const std::string& ) > complete_callback );
private:

    // ############## Functions for setup.

    void InitializeOptions();

    bool CheckFilesInOptionsManager ();

    bool CheckVenueFiles();

    void FetchLandmarks();

    // ############## Functions for localization.

    void GetLocalImageNames ( const std::vector<std::string>& image_names,
                              std::vector<std::string>* local_image_names );

    bool VerifyCameraParams ( const std::string& camera_model,
                              const std::string& params );

    // Extract SIFT feature from images in list.
    void ExtractFeature ( const std::string& camera_model_name,
                          const std::string& camera_params_csv,
                          const std::vector<std::string>& image_names );

    // Match new images with registered images.
    std::vector<image_t> MatchImages ( const std::vector<std::string>& image_names );

    // Register new images in reconstraction.
    std::vector<LocalizationResult>
    RegisterImages ( DatabaseCache* database_cache,
                     const std::vector<image_t> image_ids );

    // Convert vector of localization result into Json.
    std::string ParseLocalizationResult ( const std::string& area_name,
                                          const std::vector<LocalizationResult>& results );

    const std::string venue_name_;
    const std::string area_name_;
    double scale_to_meter_;
    std::vector<LandmarkInfo> landmarks_;

    std::shared_ptr<AzureBlobLoader> azure_blob_loader_;
    std::shared_ptr<SiftFeatureExtractor> feature_extractor_;

    OptionManager options_;
    retrieval::VisualIndex<> visual_index_;
};

#endif // LOCALIZER2_H
