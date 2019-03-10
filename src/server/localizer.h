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

#ifndef LOCALIZER_H_
#define LOCALIZER_H_

#include <iostream>

#include "AzureStorageCpplite/storage_credential.h"
#include "AzureStorageCpplite/storage_account.h"
#include "AzureStorageCpplite/blob/blob_client.h"

#include "util/misc.h"
#include "util/threading.h"
#include "util/option_manager.h"
#include "server/localization_result.h"
#include "server/landmark_info.h"

using namespace colmap;

#ifdef CUDA_ENABLED
const bool kUseOpenGL = false;
#else
const bool kUseOpenGL = true;
#endif

const std::string MODEL_PATH = "localization/aligned";
const std::string DATABASE_FILE_NAME = "localization/database.db";
const std::string LOCALIZATION_IMAGE_REPO = "localization/images";
const std::string INDEX_FILE_NAME = "localization/index.bin";
const std::string INDEX_IMAGE_LIST_PATH = "localization/database_image_list.txt";
const std::string MATCH_IMAGE_LIST_PATH = "localization/query_image_list.txt";
const std::string LANDMARK_FILE_NAME = "localization/landmarks.json";

const std::string AZURE_ACCOUNT_NAME = "5glab";
const std::string AZURE_ACCOUNT_KEY = "zWdS5g8p0KKbidI2RE5GkkB1fdeMaZ5Bg0XxOzgIyxl39DM+syVXi9LT6BNQHCp01z5kQ3YI420bcMVV9Vm3qw==";
const std::string BLOB_CONTAINER = "perceptvision";
const std::string BLOB_PREFIX = "pictures/";
const std::string BLOB_SUBFIX = ".JPG";
const int AZURE_MAX_CONCURRENCY = 10;


/**
 * This class handle task of localizing a list of new images in the corresponding venue.
 */
class Localizer : public Thread
{
public:
    Localizer(const std::string& venue_name, const std::vector<std::string>& request_image_names);

    int CollectStatus() {
        return finish_status_;
    }
    std::string CollectError() {
        cout << err_.str() << endl;
        return err_.str();
    }
    web::json::value CollectResult();

private:
    void Run();

    void LoadRequestImagesFromAzure();
    std::vector<LocalizationResult> LocalizeImages(const OptionManager& options );
    std::vector<LandmarkInfo> FetchLandmarks();

    std::shared_ptr<azure::storage_lite::blob_client_wrapper> blob_client_wrapper_;
    
    std::vector<LocalizationResult> localization_results_;
    std::vector<LandmarkInfo> landmark_infos_;
    
    double scale_to_meter_;
    
    std::string venue_name_;
    std::vector<std::string> request_image_names_;
    std::stringstream err_;
    int finish_status_;
};

#endif // LOCALIZER_H_
