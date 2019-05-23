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

#include "camera_calibrator.h"

#include "util/misc.h"
#include "base/image_reader.h"
#include "feature/extraction.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"

using namespace rapidjson;

CameraCalibrator::CameraCalibrator ( const std::shared_ptr<AzureBlobLoader> azure_blob_loader )
    :azure_blob_loader_ ( azure_blob_loader )
{
    std::cout << "Initializing camera calibrator." << std::endl;
    InitializeParentFolder();
    std::cout << "done" <<std::endl;
}

void CameraCalibrator::InitializeParentFolder()
{
    // Create the folder to hold all temporary files for camera calibration.
    CreateDirIfNotExists ( EnsureTrailingSlash ( GetWorkingDirectory() )
                           + EnsureTrailingSlash ( kCalibrationWorkspace ) );
}

void CameraCalibrator::InitializeUser ( const std::string user_id,
                                        OptionManager* options )
{
    std::string user_folder = EnsureTrailingSlash ( GetWorkingDirectory() )
                              + EnsureTrailingSlash ( kCalibrationWorkspace )
                              + EnsureTrailingSlash ( user_id );
    std::string user_images_folder = user_folder
                                     + EnsureTrailingSlash ( kCalibrationImageFolder );
    std::string user_reconstruction_folder = user_folder
            + EnsureTrailingSlash ( kCalibrationReconstructionFolder );

    CreateDirIfNotExists ( user_images_folder );
    CreateDirIfNotExists ( user_reconstruction_folder );

    // Initialize options.
    options->AddImageOptions();
    options->AddExhaustiveMatchingOptions();
    options->AddMapperOptions();
    // Add image directory to options manager.
    *options->image_path = user_images_folder;
    // Add database to options manager.
    *options->database_path = user_folder + kCalibrationDatabaseFilename;
    // Add user folder as project path.
    *options->project_path = user_folder;
    // Image reader options.
    options->image_reader->single_camera = true;
    options->image_reader->database_path = *options->database_path;
    options->image_reader->image_path = *options->image_path;
    // Mapper options.
    options->mapper->ba_refine_principal_point = true;
}

void GetLocalImageNames ( const std::vector<std::string>& image_names,
                          const std::string& user_name,
                          std::vector<std::string>* local_image_names )
{
    for ( const std::string& image_name : image_names ) {
        std::string local_image_name = EnsureTrailingSlash ( kCalibrationWorkspace )
                                       + EnsureTrailingSlash ( user_name )
                                       + EnsureTrailingSlash ( kCalibrationImageFolder )
                                       + image_name;
        local_image_names->emplace_back ( local_image_name );
        std::cout << " => " << local_image_name << std::endl;
    }
}

void CameraCalibrator::HandoverRequestProcess ( const std::string& user_name,
        const std::vector<std::string>& request_image_names,
        std::function<void ( const int, const std::string & ) > complete_callback )
{
    std::cout << "Handover" << std::endl;
    // Stop if no image to process.
    if ( request_image_names.size() == 0 ) {
        std::cout << "No image to use for calibration" << std::endl;
        complete_callback ( EXIT_FAILURE, "No image to calibrate" );
        return;
    }

    // Initialize folders and options.
    OptionManager options;
    InitializeUser ( user_name, &options );
    std::cout << "database path: " << *options.database_path << std::endl;

    // Load images.
    PrintHeading2 ( "Downloading images" );
    std::vector<std::string> local_image_names;
    GetLocalImageNames ( request_image_names, user_name, &local_image_names );
    std::future<void> download_future = std::async ( std::launch::async,
                                        &AzureBlobLoader::LoadRequestImages,
                                        azure_blob_loader_, request_image_names,
                                        local_image_names );
    download_future.get();
    std::cout << "  Done" << std::endl;

    // Extract features.
    PrintHeading2 ( "Extracting features" );
    ExtractFeatures ( &options );
    std::cout << "  Done" << std::endl;

    // Matching.
    PrintHeading2 ( "Matching images" );
    MatchImages ( &options );
    std::cout << "  Done" << std::endl;

    // Register images and optimize camera parameters.
    PrintHeading2 ( "Registering images" );
    ReconstructionManager reconstruction_manager;
    RegisterImages ( &options, &reconstruction_manager );
    std::cout << "  Done" << std::endl;

    // Read out camera parameters from database.
    PrintHeading2 ( "Reading camera parameters" );
    Database database ( *options.database_path );
    if ( reconstruction_manager.Size() == 0 ) {
        complete_callback ( EXIT_FAILURE, "unsuccessful" );
    } else {
        using rapidjson::Type;
        Reconstruction reconstruction = reconstruction_manager.Get(0);
        if(reconstruction.Cameras().size()==0){
            complete_callback(EXIT_FAILURE, "unsuccessful");
            return;
        }
        
        Camera camera = (*reconstruction.Cameras().begin()).second;

        Document document;
        document.SetObject();

        document.AddMember ( Value ( kCalibResultCameraModel.c_str(),
                                     document.GetAllocator() ).Move(),
                             Value ( camera.ModelName().c_str(),
                                     document.GetAllocator() ).Move(),
                             document.GetAllocator() );
        document.AddMember ( Value ( kCalibResultCameraParams.c_str(),
                                     document.GetAllocator() ).Move(),
                             Value ( camera.ParamsToString().c_str(),
                                     document.GetAllocator() ).Move(),
                             document.GetAllocator() );

        StringBuffer buffer;
        PrettyWriter<StringBuffer> writer ( buffer );
        document.Accept ( writer );
        complete_callback ( EXIT_SUCCESS, buffer.GetString() );
    }
    database.Close();
    
    // Clean up temporal folder.
    RemoveDirContent(*options.project_path, true);
}

void CameraCalibrator::ExtractFeatures ( OptionManager* options )
{
    ImageReaderOptions reader_options = *options->image_reader;
    reader_options.camera_model = kCalibrationDefaultCameraModel;

    SiftFeatureExtractor feature_extractor ( reader_options,
            *options->sift_extraction );

    if ( options->sift_extraction->use_gpu && kUseOpenGLx ) {
        RunThreadWithOpenGLContext ( &feature_extractor );
    } else {
        feature_extractor.Start();
        feature_extractor.Wait();
    }
}

void CameraCalibrator::MatchImages ( OptionManager* options )
{
    ExhaustiveMatchingOptions matching_options = *options->exhaustive_matching;

    ExhaustiveFeatureMatcher feature_matcher ( matching_options,
            *options->sift_matching,
            *options->database_path );

    if ( options->sift_matching->use_gpu && kUseOpenGLx ) {
        RunThreadWithOpenGLContext ( &feature_matcher );
    } else {
        feature_matcher.Start();
        feature_matcher.Wait();
    }
}

void CameraCalibrator::RegisterImages ( OptionManager* options,
                                        ReconstructionManager* reconstruction_manager)
{
    const auto sparse_path = JoinPaths ( *options->project_path,
                                         kCalibrationReconstructionFolder );

    IncrementalMapperController mapper ( options->mapper.get(),
                                         *options->image_path,
                                         *options->database_path,
                                         reconstruction_manager );
    mapper.Start();
    mapper.Wait();

    reconstruction_manager->Write ( sparse_path, options );
}
