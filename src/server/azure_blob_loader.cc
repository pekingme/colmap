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

#include "azure_blob_loader.h"

#include <assert.h>

AzureBlobLoader::AzureBlobLoader()
{
    // Create Azure storage blob client.
    std::shared_ptr<azure::storage_lite::shared_key_credential> credential =
        std::make_shared<azure::storage_lite::shared_key_credential> ( kAzureAccountName, kAzureAccountKey );
    std::shared_ptr<azure::storage_lite::storage_account> account =
        std::make_shared<azure::storage_lite::storage_account> ( kAzureAccountName, credential, true );
    std::shared_ptr<azure::storage_lite::blob_client> blob_client =
        std::make_shared<azure::storage_lite::blob_client> ( account, kAzureMaxConcurrency );
    blob_client_wrapper_ = std::make_shared<azure::storage_lite::blob_client_wrapper> ( blob_client );

    // Make sure blob container exists.
    assert ( blob_client_wrapper_->container_exists ( BLOB_CONTAINER_X ) );
}

void AzureBlobLoader::LoadRequestImages (
    const std::vector<std::string>& image_names,
    const std::vector<std::string>& local_image_names )
{
    assert ( blob_image_names.size() == local_image_names.size() );
    std::vector<std::future<void>*> futures;

    for ( size_t i=0; i<image_names.size(); i++ ) {
        std::cout << "Donwload image " << image_names[i] << std::endl;
        std::string blob_image_name = EnsureTrailingSlash ( kBlobPrefix ) + image_names[i];
        std::string local_image_path = local_image_names[i];
        std::future<void> future = std::async ( std::launch::async,
                                                &AzureBlobLoader::DownloadImage, this,
                                                blob_image_name,local_image_path );
        futures.emplace_back ( &future );
    }

    for ( size_t i=0; i<image_names.size(); i++ ) {
        if(futures[i]->valid()) {
            std::cout << image_names[i] << " valid" << std::endl;
            futures[i]->get();
        } else {
            std::cout << image_names[i] << " invalid" << std::endl;
        }
    }
}

void AzureBlobLoader::DownloadImage ( const std::string& blob_name,
                                      const std::string& local_name )
{
    // Make sure blob exists before downloading.
    assert ( blob_client_wrapper_->blob_exists ( kBlobContainer, blob_image_name ) );
    time_t last_modified;
    blob_client_wrapper_->download_blob_to_file ( kBlobContainer, blob_name,
            local_name, last_modified );
    if ( errno != 0 ) {
        std::cout << "Download error code: " << errno << std::endl;
    } else {
        std::cout << "Downloaded: \"" << blob_name << "\" --> "
                  << "\""<< local_name << "\"" << std::endl;
    }
}
