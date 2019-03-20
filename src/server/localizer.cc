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

#include "server/localizer.h"

#include <QApplication>

#include "util/misc.h"
#include "base/reconstruction.h"
#include "base/image_reader.h"
#include "base/camera_models.h"
#include "base/database_cache.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "controllers/incremental_mapper.h"
#include "server/cpprest_import.h"

bool VerifyCameraParams ( const std::string& camera_model,
                          const std::string& params )
{
    if ( !ExistsCameraModelWithName ( camera_model ) ) {
        std::cerr << "ERROR: Camera model does not exist" << std::endl;
        return false;
    }

    const std::vector<double> camera_params = CSVToVector<double> ( params );
    const int camera_model_id = CameraModelNameToId ( camera_model );

    if ( camera_params.size() > 0 &&
            !CameraModelVerifyParams ( camera_model_id, camera_params ) ) {
        std::cerr << "ERROR: Invalid camera parameters" << std::endl;
        return false;
    }
    return true;
}

bool CheckFilesInOptionManager ( std::stringstream& err_, const OptionManager& options )
{
    bool valid = true;
    if ( !ExistsFile ( *options.database_path ) ) {
        err_ << "Database file not found at" << endl
             << "\t" << *options.database_path << endl;
        valid = false;
    }
    if ( !ExistsFile ( options.vocab_tree_matching->vocab_tree_path ) ) {
        err_ << "Vocabulary tree (index file) not found at" << endl
             << "\t" << options.vocab_tree_matching->vocab_tree_path << endl;
        valid = false;
    }
    if ( !ExistsFile ( options.vocab_tree_matching->index_list_path ) ) {
        err_ << "Database image list file not found at" << endl
             << "\t" << options.vocab_tree_matching->index_list_path << endl;
        valid = false;
    }
    if ( !ExistsFile ( options.vocab_tree_matching->match_list_path ) ) {
        err_ << "Query image list file not found at" << endl
             << "\t" << options.vocab_tree_matching->match_list_path << endl;
        valid = false;
    }
    return valid;
}

void CopyImageNamesToQueryListFile ( const std::string& query_file_name,
                                     const std::vector<std::string>& image_names )
{
    std::ofstream query_file ( query_file_name, std::ofstream::trunc );
    for ( const auto& image_name : image_names ) {
        query_file << image_name << std::endl;
    }
    query_file.close();
}

inline std::string GetBlobImageName ( const std::string& request_image_name )
{
    return EnsureTrailingSlash ( BLOB_PREFIX ) + request_image_name ;
}

inline std::string GetLocalImagePath ( const std::string& request_image_name, const std::string& venue_name )
{
    return EnsureTrailingSlash ( venue_name ) + EnsureTrailingSlash ( LOCALIZATION_IMAGE_REPO ) + request_image_name ;
}

inline std::string GetLocalImageName ( const std::string& request_image_name )
{
    return request_image_name + BLOB_SUBFIX;
}

Localizer::Localizer ( const std::string& venue_name, const std::vector<std::string>& request_image_names )
    : venue_name_ ( EnsureTrailingSlash ( venue_name ) ), request_image_names_ ( request_image_names )
{
    // Create Azure storage blob client.
    std::shared_ptr<azure::storage_lite::shared_key_credential> credential =
        std::make_shared<azure::storage_lite::shared_key_credential> ( AZURE_ACCOUNT_NAME, AZURE_ACCOUNT_KEY );
    std::shared_ptr<azure::storage_lite::storage_account> account =
        std::make_shared<azure::storage_lite::storage_account> ( AZURE_ACCOUNT_NAME, credential, true );
    auto blob_client = std::make_shared<azure::storage_lite::blob_client> ( account, AZURE_MAX_CONCURRENCY );
    blob_client_wrapper_ = std::make_shared<azure::storage_lite::blob_client_wrapper> ( blob_client );
    // Make sure blob container exists.
    assert ( blob_client_wrapper_->container_exists ( BLOB_CONTAINER ) );
}

void Localizer::CalculateLocation(const std::string& camera_model_name,
                                  const std::string& camera_params_csv) {
    cout << "Venue name: " << venue_name_ << endl;
    // Lording images from Azure Storage.
    if(!LoadRequestImagesFromAzure()) {
        cout << "Loading images from Azure failed." << endl;
        finish_status_ = EXIT_FAILURE;
        return;
    }

    // Prepare COLMAP options.
    SetupOptions(camera_model_name, camera_params_csv);

    // Check and prepare local files.
    SetupLocalFiles();

    // Register query images in mapper.
    LocalizeImages();
    
    // Fetch landmarks.
    FetchLandmarks();
}

bool Localizer::LoadRequestImagesFromAzure()
{
    for ( const auto& request_image_name : request_image_names_ ) {
        std::string blob_image_name = GetBlobImageName ( request_image_name );
        std::string local_image_path = GetLocalImagePath ( request_image_name, venue_name_ );
        std::cout << "Download: \"" << blob_image_name << "\" --> "
                  << "\""<< local_image_path << "\"" << std::endl;

        // Make sure blob exists before downloading.
        assert ( blob_client_wrapper_->blob_exists ( BLOB_CONTAINER, blob_image_name ) );
        time_t last_modified;
        blob_client_wrapper_->download_blob_to_file ( BLOB_CONTAINER, blob_image_name, local_image_path, last_modified );
        if(errno != 0) {
            std::cout << "Download error code: " << errno << std::endl;
            return false;
        }
    }
    return true;
}

void Localizer::SetupOptions(const std::string& camera_model_name,
                             const std::string& camera_params_csv)
{
    options_.AddImageOptions();
    options_.AddVocabTreeMatchingOptions();
    options_.AddMapperOptions();

    // Add database to options.
    *options_.database_path = venue_name_+DATABASE_FILE_NAME;
    // Update image reader options.
    options_.image_reader->camera_model = camera_model_name;
    options_.image_reader->camera_params = camera_params_csv;
    options_.image_reader->single_camera = true;
    options_.image_reader->database_path = *options_.database_path;
    options_.image_reader->image_list.clear();
    for ( const string& request_image_name : request_image_names_ ) {
        options_.image_reader->image_list.push_back ( request_image_name );
    }
    options_.image_reader->image_path = venue_name_+LOCALIZATION_IMAGE_REPO;
    // Add images to matching options.
    options_.vocab_tree_matching->num_images = 10;
    options_.vocab_tree_matching->vocab_tree_path = venue_name_+INDEX_FILE_NAME;
    options_.vocab_tree_matching->index_list_path = venue_name_+INDEX_IMAGE_LIST_PATH;
    options_.vocab_tree_matching->match_list_path = venue_name_+MATCH_IMAGE_LIST_PATH;
}

void Localizer::SetupLocalFiles()
{
    if ( !CheckFilesInOptionManager ( err_, options_ ) ) {
        finish_status_ = EXIT_FAILURE;
        return;
    }
    if ( !ExistsDir ( venue_name_+MODEL_PATH ) ) {
        err_ << "Model directory not found at" << endl
             << "\t" << venue_name_+MODEL_PATH << endl;
        finish_status_ = EXIT_FAILURE;
        return;
    }
    if ( !ExistsFile ( venue_name_+LANDMARK_FILE_NAME ) ) {
        err_ << "Landmark file not found at" << endl
             << "\t" << venue_name_+LANDMARK_FILE_NAME << endl;
        finish_status_ = EXIT_FAILURE;
        return;
    }
    CopyImageNamesToQueryListFile ( options_.vocab_tree_matching->match_list_path,
                                    options_.image_reader->image_list );
}

void Localizer::LocalizeImages ()
{
    ImageReaderOptions reader_options = *options_.image_reader;

    localization_results_.clear();

    if ( reader_options.image_list.size() == 0 ) {
        err_ << "No images to estimate" << endl;
        finish_status_ = EXIT_FAILURE;
        return;
    }

    if ( !VerifyCameraParams ( options_.image_reader->camera_model,
                               options_.image_reader->camera_params ) ) {
        err_ << "Bad camera parameters" << endl;
        finish_status_ = EXIT_FAILURE;
        return;
    }

    // Extract SIFT features from images in list.
    SiftFeatureExtractor feature_extractor ( reader_options,
            *options_.sift_extraction );

    if ( options_.sift_extraction->use_gpu && kUseOpenGL ) {
        RunThreadWithOpenGLContext ( &feature_extractor );
    } else {
        feature_extractor.Start();
        feature_extractor.Wait();
    }
    
    // Add all request image names in to a set.
    std::unordered_set<std::string> request_image_names;
    request_image_names.insert(reader_options.image_list.begin(),
                               reader_options.image_list.end());

    // VocabTree matching to all registered images.
    VocabTreeFeatureMatcher feature_matcher ( *options_.vocab_tree_matching,
            *options_.sift_matching,
            *options_.database_path );

    if ( options_.sift_matching->use_gpu && kUseOpenGL ) {
        RunThreadWithOpenGLContext ( &feature_matcher );
    } else {
        feature_matcher.Start();
        feature_matcher.Wait();
    }

    // Load database after feature extraction and matching.
    PrintHeading1 ( "Loading database" );
    Database database ( *options_.database_path );
    DatabaseCache database_cache;
    {
        Timer timer;
        timer.Start();
        const size_t min_num_matches =
            static_cast<size_t> ( options_.mapper->min_num_matches );
        database_cache.Load ( database, min_num_matches,
                              options_.mapper->ignore_watermarks,
                              options_.mapper->image_names );
        std::cout << std::endl;
        timer.PrintMinutes();
    }
    std::cout << std::endl;

    Reconstruction reconstruction;
    reconstruction.Read ( venue_name_+MODEL_PATH );

    std::vector<image_t> clean_up_image_ids;
    std::vector<camera_t> clean_up_camera_ids;

    IncrementalMapper mapper ( &database_cache );
    mapper.BeginReconstruction ( &reconstruction );

    const auto mapper_options = options_.mapper->Mapper();

    localization_results_.clear();
    for ( const auto& image : reconstruction.Images() ) {
        if(request_image_names.count(image.second.Name()) == 0){
            continue;
        }
        
        if ( image.second.IsRegistered() ) {
            PrintHeading1 ( "Image #" + std::to_string ( image.first ) +
                            " is registered. Previous pose is assigned." );

            LocalizationResult result ( image.second, true );
            localization_results_.push_back ( result );
        } else {
            PrintHeading1 ( "Registering image #" + std::to_string ( image.first ) + " (" +
                            std::to_string ( reconstruction.NumRegImages() + 1 ) + ")" );

            std::cout << "  => Image sees " << image.second.NumVisiblePoints3D()
                      << " / " << image.second.NumObservations() << " points"
                      << std::endl;

            if ( mapper.RegisterNextImage ( mapper_options, image.first ) ) {
                std::cout << "  => Succeed" << std::endl;
                LocalizationResult result ( image.second, true );
                localization_results_.push_back ( result );
            } else {
                std::cout << "  => Failed" << std::endl;
                LocalizationResult result ( image.second, false );
                localization_results_.push_back ( result );
            }

            //clean_up_image_ids.emplace_back ( image.first );
            //clean_up_camera_ids.emplace_back ( image.second.CameraId() );
        }
    }
    
    std::cout << "DEBUG: finished registration" << std::endl;

    const bool kDiscardReconstruction = true;
    mapper.EndReconstruction ( kDiscardReconstruction );

    // Clean up database.
    for ( const auto& image_id : clean_up_image_ids ) {
        database.DeleteImage ( image_id );
        for ( const auto& image : reconstruction.Images() ) {
            if ( database.ExistsMatches ( image_id, image.first ) ) {
                database.DeleteMatches ( image_id, image.first );
                if ( database.ExistsInlierMatches ( image_id, image.first ) ) {
                    database.DeleteInlierMatches ( image_id, image.first );
                }
            }
        }
    }
    for(const auto& camera_id : clean_up_camera_ids) {
        database.DeleteCamera(camera_id);
    }
    
    finish_status_ = EXIT_SUCCESS;
}

void Localizer::FetchLandmarks()
{
    boost::property_tree::ptree ptree;
    // Parse landmark json file to property tree.
    boost::property_tree::read_json ( venue_name_+LANDMARK_FILE_NAME, ptree );

    if ( ptree.count ( "scale_to_meter" ) == 0 ) {
        scale_to_meter_ = 1.0;
    } else {
        scale_to_meter_ = ptree.get<double> ( "scale_to_meter" );
    }

    landmark_infos_.clear();
    // Translate property tree to landmark info.
    for ( const auto& landmark_pt : ptree.get_child ( "landmarks" ) ) {
        std::string landmark_name = landmark_pt.second.get<string> ( "name" );
        double landmark_pos_x = landmark_pt.second.get<double> ( "x" );
        double landmark_pos_y = landmark_pt.second.get<double> ( "y" );
        double landmark_pos_z = landmark_pt.second.get<double> ( "z" );
        landmark_infos_.emplace_back ( landmark_name, landmark_pos_x, landmark_pos_y, landmark_pos_z );
    }
}

web::json::value Localizer::CollectResult()
{
    web::json::value result = web::json::value::object();

    web::json::value landmark_infos_value = web::json::value::array();
    for ( size_t i=0; i<landmark_infos_.size(); i++ ) {
        landmark_infos_value[i] = landmark_infos_[i].Scale ( scale_to_meter_ ).AsJSON();
    }
    result["landmark_infos"] = landmark_infos_value;

    web::json::value localization_results_value = web::json::value::array();
    for ( size_t i=0; i<localization_results_.size(); i++ ) {
        localization_results_value[i] = localization_results_[i].Scale ( scale_to_meter_ ).AsJSON();
        cout << localization_results_[i] << endl;
    }
    result["localization_result"] = localization_results_value;
    
    return result;
}
