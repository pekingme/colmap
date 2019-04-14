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

#ifndef AZUREBLOBLOADER_H
#define AZUREBLOBLOADER_H

#include <string>
#include <vector>
#include <thread>
#include <future>

#include "util/threading.h"
#include "util/misc.h"

#include "AzureStorageCpplite/storage_credential.h"
#include "AzureStorageCpplite/storage_account.h"
#include "AzureStorageCpplite/blob/blob_client.h"

using namespace colmap;

const std::string kAzureAccountName = "5glab";
const std::string kAzureAccountKey = "zWdS5g8p0KKbidI2RE5GkkB1fdeMaZ5Bg0XxOzgIyxl39DM+syVXi9LT6BNQHCp01z5kQ3YI420bcMVV9Vm3qw==";
const std::string kBlobContainer = "perceptvision";
const std::string kBlobPrefix = "pictures/";
const int kAzureMaxConcurrency = 10;

/**
 * @todo write docs
 */
class AzureBlobLoader
{
public:
    AzureBlobLoader();

    void LoadRequestImages(const std::vector<std::string>& image_names, 
                           const std::vector<std::string>& local_image_names);

private:
    void DownloadImage(const std::string& blob_name, const std::string& local_name);
    
    std::shared_ptr<azure::storage_lite::blob_client_wrapper> blob_client_wrapper_;
};

#endif // AZUREBLOBLOADER_H
