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

#include "localizer2.h"

#include "feature/matching.h"
#include "base/camera_models.h"
#include "controllers/incremental_mapper.h"

Localizer2::Localizer2 ( const std::string& venue_name,
                         const std::shared_ptr<AzureBlobLoader> azure_blob_loader )
    :venue_name_ ( EnsureTrailingSlash ( venue_name ) ), azure_blob_loader_ ( azure_blob_loader )
{
    std::cout << "Initializing localizer for " << venue_name << std::endl;

    InitializeOptions();

    PrintHeading2 ( "Check files for "+venue_name_ );
    if ( !CheckVenueFiles() ) {
        throw "Files are missing.";
    } else {
        std::cout << "  Done" << std::endl;
    }

    PrintHeading2 ( "Loading landmarks" );
    FetchLandmarks();
    std::cout << "  Done" << std::endl;

    // TODO setup feature extraction cache
    // TODO setup feature matching cache
    // TODO setup database cache
    // TODO setup reconstruction

}

void Localizer2::InitializeOptions()
{
    options_.AddImageOptions();
    options_.AddVocabTreeMatchingOptions();
    options_.AddMapperOptions();
    // Database.
    *options_.database_path = venue_name_+kDatabaseFilename;
    // Image reader options.
    options_.image_reader->single_camera = true;
    options_.image_reader->database_path = *options_.database_path;
    options_.image_reader->image_path = venue_name_+kLocalizationImageRepo;
    // Vocab tree matching options.
    options_.vocab_tree_matching->num_images = 10;
    options_.vocab_tree_matching->vocab_tree_path = venue_name_+kIndexFilename;
    options_.vocab_tree_matching->index_list_path = venue_name_+kIndexImageListPath;
}

bool Localizer2::CheckVenueFiles()
{
    bool valid = true;
    if ( !CheckFilesInOptionsManager() ) {
        valid = false;
    }
    if ( !ExistsDir ( venue_name_+kModelPath ) ) {
        std::cerr << "Model directory not found at" << std::endl
                  << "\t" << venue_name_+kModelPath << std::endl;
        valid = false;
    }
    if ( !ExistsFile ( venue_name_+kLandmarkFilename ) ) {
        std::cerr << "Landmark file not found at" << std::endl
                  << "\t" << venue_name_+kLandmarkFilename << std::endl;
        valid = false;
    }

    return valid;
}

bool Localizer2::CheckFilesInOptionsManager()
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

void Localizer2::FetchLandmarks()
{
    boost::property_tree::ptree ptree;
    // Parse landmark json file to property tree.
    boost::property_tree::read_json ( venue_name_+kLandmarkFilename, ptree );

    if ( ptree.count ( "scale_to_meter" ) == 0 ) {
        scale_to_meter_ = 1.0;
    } else {
        scale_to_meter_ = ptree.get<double> ( "scale_to_meter" );
    }

    landmarks_.clear();
    // Translate property tree to landmark info.
    for ( const auto& landmark_pt : ptree.get_child ( "landmarks" ) ) {
        std::string landmark_name = landmark_pt.second.get<string> ( "name" );
        double landmark_pos_x = landmark_pt.second.get<double> ( "x" );
        double landmark_pos_y = landmark_pt.second.get<double> ( "y" );
        double landmark_pos_z = landmark_pt.second.get<double> ( "z" );
        landmarks_.emplace_back ( landmark_name, landmark_pos_x, landmark_pos_y, landmark_pos_z );
    }
}

std::pair<int, web::json::value>
Localizer2::Localize ( const std::string& camera_model_name,
                       const std::string& camera_params_csv,
                       const std::vector<std::string>& image_names )
{
    // Stop if no image to process.
    if ( image_names.size() == 0 ) {
        std::cout << "No image to estimate" << std::endl;
        return std::make_pair ( EXIT_FAILURE, web::json::value() );
    }

    // Stop if camera model is invalid.
    if ( !VerifyCameraParams ( camera_model_name, camera_params_csv ) ) {
        std::cout << "Camera model is not valid" << std::endl;
        return std::make_pair ( EXIT_FAILURE, web::json::value() );
    }

    // load images
    PrintHeading2 ( "Downloading images" );
    std::vector<std::string> local_image_names;
    GetLocalImageNames ( image_names, &local_image_names );
    std::future<void> download_future = std::async ( std::launch::async, &AzureBlobLoader::LoadRequestImages, azure_blob_loader_, image_names, local_image_names );
    download_future.get();
    std::cout << "  Done" << std::endl;

    // feature extraction
    PrintHeading2 ( "Extracting features" );
    ExtractFeature ( camera_model_name, camera_params_csv, image_names );
    std::cout << "  Done" << std::endl;

    // matching
    PrintHeading2 ( "Matching images" );
    std::vector<image_t> image_ids = MatchImages ( image_names );
    std::cout << "  Done" << std::endl;

    // load databbase cache
    PrintHeading2 ( "Loading database" );
    Database database ( *options_.database_path );
    DatabaseCache database_cache;
    {
        Timer timer;
        timer.Start();
        const size_t min_num_matches =
            static_cast<size_t> ( options_.mapper->min_num_matches );
        database_cache.Load ( database, min_num_matches,options_.mapper->ignore_watermarks, options_.mapper->image_names );
        std::cout << std::endl;
        timer.PrintMinutes();
        std::cout << std::endl;
    }

    // registration
    PrintHeading2 ( "Registering images" );
    std::vector<LocalizationResult> results =
        RegisterImages ( &database_cache, image_ids );
    std::cout << "  Done" << std::endl;

    // clean up database TODO this should be avoid.
    PrintHeading2 ( "Cleaning database" );
    std::unordered_set<camera_t> clean_up_camera_ids;
    for ( const image_t image_id : image_ids ) {
        std::cout << "Deleting image " << image_id << std::endl;
        const Image& image = database.ReadImage ( image_id );
        clean_up_camera_ids.insert ( image.CameraId() );

        database.DeleteImage ( image_id );
        for ( const auto& image : database_cache.Images() ) {
            if ( database.ExistsMatches ( image_id, image.first ) ) {
                database.DeleteMatches ( image_id, image.first );
                if ( database.ExistsInlierMatches ( image_id, image.first ) ) {
                    database.DeleteInlierMatches ( image_id, image.first );
                }
            }
        }
    }
    for ( const camera_t camera_id : clean_up_camera_ids ) {
        std::cout << "Deleting camera " << camera_id << std::endl;
        database.DeleteCamera ( camera_id );
    }
    database.Close();
    std::cout << "  Done" << std::endl;

    // parse result to json
    web::json::value json_results = ParseLocalizationResult ( results );
    return std::make_pair ( EXIT_SUCCESS, json_results );
}

void Localizer2::GetLocalImageNames (
    const std::vector<std::string>& image_names,
    std::vector<std::string>* local_image_names )
{
    for ( const std::string& image_name : image_names ) {
        std::string local_image_name = EnsureTrailingSlash ( venue_name_ ) + EnsureTrailingSlash ( kLocalizationImageRepo ) + image_name;
        local_image_names->emplace_back ( local_image_name );
        std::cout << " => " << local_image_name << std::endl;
    }
}

bool Localizer2::VerifyCameraParams ( const std::string& camera_model,
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

void Localizer2::ExtractFeature ( const std::string& camera_model_name,
                                  const std::string& camera_params_csv,
                                  const std::vector<std::string>& image_names )
{
    ImageReaderOptions reader_options = *options_.image_reader;

    reader_options.camera_model = camera_model_name;
    reader_options.camera_params = camera_params_csv;
    reader_options.image_list.clear();
    for ( const std::string& image_name : image_names ) {
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

std::vector<image_t> Localizer2::MatchImages ( const std::vector<std::string>& image_names )
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
Localizer2::RegisterImages ( DatabaseCache* database_cache,
                             const std::vector<image_t> image_ids )
{
    std::vector<LocalizationResult> results;

    Reconstruction reconstruction;
    reconstruction.Read ( venue_name_+kModelPath );

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

web::json::value Localizer2::ParseLocalizationResult (
    const std::vector<LocalizationResult>& results )
{
    web::json::value json = web::json::value::object();

    web::json::value localization_json_value = web::json::value::array();
    for ( size_t i=0; i<results.size(); i++ ) {
        localization_json_value[i] = results[i].Scale ( scale_to_meter_ ).AsJSON();
        cout << results[i] << endl;
    }
    json["localization_result"] = localization_json_value;

    return json;
}
