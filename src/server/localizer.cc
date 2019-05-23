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

#include "localizer.h"

#include "feature/matching.h"
#include "base/camera_models.h"
#include "controllers/incremental_mapper.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/document.h"

using namespace rapidjson;

Localizer::Localizer ( const std::string& venue_name, const std::string& area_name,
                       const std::shared_ptr<AzureBlobLoader> azure_blob_loader)
    :venue_name_ ( EnsureTrailingSlash ( venue_name ) ), area_name_ ( EnsureTrailingSlash ( area_name ) ),
     azure_blob_loader_ ( azure_blob_loader )
{
    std::cout << "Initializing localizer for " << venue_name_ << area_name_ << std::endl;

    InitializeOptions();

    PrintHeading2 ( "Check files for "+venue_name_+area_name_ );
    if ( !CheckVenueFiles() ) {
        throw "Files are missing.";
    } else {
        std::cout << "  Done" << std::endl;
    }
}

void Localizer::InitializeOptions()
{
    options_.AddImageOptions();
    options_.AddVocabTreeMatchingOptions();
    options_.AddMapperOptions();
    // Database.
    *options_.database_path = venue_name_+area_name_+kLocalizationDatabaseFilename;
    // Image reader options.
    options_.image_reader->single_camera = true;
    options_.image_reader->database_path = *options_.database_path;
    options_.image_reader->image_path = venue_name_+area_name_+kLocalizationImagesFolder;
    // Vocab tree matching options.
    options_.vocab_tree_matching->num_images = 10;
    options_.vocab_tree_matching->vocab_tree_path = venue_name_+area_name_+kLocalizationIndexingFilename;
    options_.vocab_tree_matching->index_list_path = venue_name_+area_name_+kLocalizationIndexingImageListFilename;
}

bool Localizer::CheckVenueFiles()
{
    bool valid = true;
    if ( !CheckFilesInOptionsManager() ) {
        valid = false;
    }
    if ( !ExistsDir ( venue_name_+area_name_+kLocalizationReconstructionFolder ) ) {
        std::cerr << "Model directory not found at" << std::endl
                  << "\t" << venue_name_+area_name_+kLocalizationReconstructionFolder << std::endl;
        valid = false;
    }

    return valid;
}

bool Localizer::CheckFilesInOptionsManager()
{
    bool valid = true;
    if ( !ExistsFile ( *options_.database_path ) ) {
        std::cerr << "Database file not found at" << std::endl
                  << "\t" << *options_.database_path << std::endl;
        valid = false;
    }
    if ( !ExistsFile ( options_.vocab_tree_matching->vocab_tree_path ) ) {
        std::cerr << "Vocabulary tree (index file) not found at" << std::endl
                  << "\t" << options_.vocab_tree_matching->vocab_tree_path << std::endl;
        valid = false;
    }
    if ( !ExistsFile ( options_.vocab_tree_matching->index_list_path ) ) {
        std::cerr << "Database image list file not found at" << std::endl
                  << "\t" << options_.vocab_tree_matching->index_list_path << std::endl;
        valid = false;
    }
    return valid;
}

void Localizer::HandoverRequestProcess ( const std::string& camera_model_name,
        const std::string& camera_params_csv,
        const std::vector<std::string>& request_image_names,
        std::function<void ( const int, const std::string& ) > complete_callback )
{
    std::cout << "Handover" << std::endl;
    // Stop if no image to process.
    if ( request_image_names.size() == 0 ) {
        std::cout << "No image to estimate" << std::endl;
        complete_callback ( EXIT_FAILURE, "No image to estimate" );
        return;
    }

    // Stop if camera model is invalid.
    if ( !VerifyCameraParams ( camera_model_name, camera_params_csv ) ) {
        std::cout << "Camera model is not valid" << std::endl;
        complete_callback ( EXIT_FAILURE, "Camera model is not valid" );
        return;
    }

    // load images
    PrintHeading2 ( "Downloading images" );
    std::vector<std::string> local_image_names;
    GetLocalImageNames ( request_image_names, &local_image_names );
    std::future<void> download_future = std::async ( std::launch::async, &AzureBlobLoader::LoadRequestImages,
                                        azure_blob_loader_, request_image_names, local_image_names );
    download_future.get();
    std::cout << "  Done" << std::endl;

    // feature extraction, require in main thread to use OpenGL.
    PrintHeading2 ( "Extracting features" );
    ExtractFeature ( camera_model_name, camera_params_csv, request_image_names );
    std::cout << "  Done" << std::endl;

    // matching, require in main thread to use OpenGL.
    PrintHeading2 ( "Matching images" );
    std::vector<image_t> image_ids = MatchImages ( request_image_names );
    std::cout << "  Done" << std::endl;

    // Create a thread to process localization.
    std::future<void> future = std::async ( std::launch::async, [complete_callback, image_ids, this] {
        // load databbase cache
        PrintHeading2 ( "Loading database" );
        Database database ( *options_.database_path );
        DatabaseCache* database_cache = new DatabaseCache();
        {
            Timer timer;
            timer.Start();
            const size_t min_num_matches =
            static_cast<size_t> ( options_.mapper->min_num_matches );
            database_cache->Load ( database, min_num_matches, options_.mapper->ignore_watermarks, options_.mapper->image_names );
            std::cout << std::endl;
            timer.PrintMinutes();
            std::cout << std::endl;
        }

        // registration
        PrintHeading2 ( "Registering images" );
        std::vector<LocalizationResult> results =
        RegisterImages ( database_cache, image_ids );
        std::cout << "  Done" << std::endl;

        // clean up database.
        PrintHeading2 ( "Cleaning database" );
        std::unordered_set<camera_t> clean_up_camera_ids;
        for ( const image_t image_id : image_ids )
        {
            std::cout << "Deleting image " << image_id << std::endl;
            const Image& image = database.ReadImage ( image_id );
            clean_up_camera_ids.insert ( image.CameraId() );

            database.DeleteImage ( image_id );
            for ( const auto& image : database_cache->Images() ) {
                if ( database.ExistsMatches ( image_id, image.first ) ) {
                    database.DeleteMatches ( image_id, image.first );
                    if ( database.ExistsInlierMatches ( image_id, image.first ) ) {
                        database.DeleteInlierMatches ( image_id, image.first );
                    }
                }
            }
        }
        for ( const camera_t camera_id : clean_up_camera_ids )
        {
            std::cout << "Deleting camera " << camera_id << std::endl;
            database.DeleteCamera ( camera_id );
        }
        database.Close();
        database_cache->Unload();
        delete database_cache;
        
        std::cout << "  Done" << std::endl;

        // parse result to json
        string json_string = ParseLocalizationResult ( results );
        complete_callback(EXIT_SUCCESS, json_string);
    } );

    // Create a thread to check timeout.
    std::async ( std::launch::async, [complete_callback, &future] {
        std::chrono::seconds process_duration ( 30 );

        if ( future.wait_for ( process_duration ) == std::future_status::timeout )
        {
            complete_callback ( EXIT_FAILURE, "Server timeout (30 seconds)" );
        }
    } );
}

void Localizer::GetLocalImageNames (
    const std::vector<std::string>& image_names,
    std::vector<std::string>* local_image_names )
{
    for ( const std::string& image_name : image_names ) {
        std::string local_image_name = venue_name_+area_name_ + EnsureTrailingSlash ( kLocalizationImagesFolder ) + image_name;
        local_image_names->emplace_back ( local_image_name );
        std::cout << " => " << local_image_name << std::endl;
    }
}

bool Localizer::VerifyCameraParams ( const std::string& camera_model,
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

void Localizer::ExtractFeature ( const std::string& camera_model_name,
                                 const std::string& camera_params_csv,
                                 const std::vector<std::string>& image_names )
{
    std::cout << camera_model_name<<std::endl;
    std::cout << camera_params_csv<<std::endl;

    ImageReaderOptions reader_options = *options_.image_reader;

    reader_options.camera_model = camera_model_name;
    reader_options.camera_params = camera_params_csv;
    reader_options.image_list.clear();
    for ( const std::string& image_name : image_names ) {
        std::cout << image_name<<std::endl;
        reader_options.image_list.emplace_back ( image_name );
    }

    SiftFeatureExtractor feature_extractor ( reader_options,
            *options_.sift_extraction );


    if ( options_.sift_extraction->use_gpu && kUseOpenGLx ) {
        RunThreadWithOpenGLContext ( &feature_extractor );
    } else {
        feature_extractor.Start();
        feature_extractor.Wait();
    }
}

std::vector<image_t> Localizer::MatchImages ( const std::vector<std::string>& image_names )
{
    VocabTreeMatchingOptions matching_options = *options_.vocab_tree_matching;

    matching_options.match_list.clear();
    for ( const std::string& image_name:image_names ) {
        matching_options.match_list.emplace_back ( image_name );
    }

    std::vector<image_t> image_ids;
    VocabTreeFeatureMatcher feature_matcher ( matching_options,
            *options_.sift_matching, *options_.database_path, &image_ids );

    if ( options_.sift_matching->use_gpu && kUseOpenGLx ) {
        RunThreadWithOpenGLContext ( &feature_matcher );
    } else {
        feature_matcher.Start();
        feature_matcher.Wait();
    }

    return image_ids;
}

std::vector<LocalizationResult>
Localizer::RegisterImages ( DatabaseCache* database_cache,
                            const std::vector<image_t> image_ids )
{
    std::vector<LocalizationResult> results;

    Reconstruction reconstruction;
    reconstruction.Read ( venue_name_+area_name_+kLocalizationReconstructionFolder );

    IncrementalMapper mapper ( database_cache );
    mapper.BeginReconstruction ( &reconstruction );

    const auto mapper_options = options_.mapper->Mapper();

    for ( const image_t image_id : image_ids ) {
        if ( reconstruction.ExistsImage ( image_id ) ) {
            const Image& image = reconstruction.Image ( image_id );
            if ( image.IsRegistered() ) {
                PrintHeading1 ( "Image #"+std::to_string ( image_id )+
                                " is regiestered. Previous pose is assigned." );
                results.emplace_back ( image, true );
            } else {
                PrintHeading1 ( "Registering image #"+std::to_string ( image_id ) );

                std::cout <<"  => Image sees "<<image.NumVisiblePoints3D()
                          <<" / " <<image.NumObservations() << " points" <<std::endl;

                if ( mapper.RegisterNextImage ( mapper_options, image_id ) ) {
                    std::cout << "  => Succeed" << std::endl;
                    results.emplace_back ( image, true );
                } else {
                    std::cout << "  => Failed" << std::endl;
                    results.emplace_back ( image, false );
                }
            }
        }
    }

    const bool kDiscardReconstruction = true;
    mapper.EndReconstruction ( kDiscardReconstruction );

    return results;
}

std::string Localizer::ParseLocalizationResult (
    const std::vector<LocalizationResult>& results )
{
    using rapidjson::Type;

    Document document;
    document.SetObject();

    Value localization_results ( kArrayType );
    for ( size_t i=0; i<results.size(); i++ ) {
        localization_results.PushBack (
            results[i].AsJSON ( &document ),
            document.GetAllocator() );
    }
    document.AddMember ( "localization_result", localization_results,
                         document.GetAllocator() );

    StringBuffer buffer;
    PrettyWriter<StringBuffer> writer ( buffer );
    document.Accept ( writer );

    return buffer.GetString();
}
