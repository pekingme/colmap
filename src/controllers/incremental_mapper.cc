// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "controllers/incremental_mapper.h"

#include "util/misc.h"

namespace colmap
{
namespace
{

size_t TriangulateImage ( const IncrementalMapperOptions& options,
                          const Image& image, IncrementalMapper* mapper )
{
    std::cout << "  => Continued observations: " << image.NumPoints3D()
              << std::endl;
    const size_t num_tris =
        mapper->TriangulateImage ( options.Triangulation(), image.ImageId() );
    std::cout << "  => Added observations: " << num_tris << std::endl;
    return num_tris;
}

void AdjustGlobalBundle ( const IncrementalMapperOptions& options,
                          IncrementalMapper* mapper )
{
    BundleAdjustmentOptions custom_options = options.GlobalBundleAdjustment();

    const size_t num_reg_images = mapper->GetReconstruction().NumRegImages();

    // Use stricter convergence criteria for first registered images.
    const size_t kMinNumRegImages = 10;
    if ( num_reg_images < kMinNumRegImages ) {
        custom_options.solver_options.function_tolerance /= 10;
        custom_options.solver_options.gradient_tolerance /= 10;
        custom_options.solver_options.parameter_tolerance /= 10;
        custom_options.solver_options.max_num_iterations *= 2;
        custom_options.solver_options.max_linear_solver_iterations = 200;
    }

    PrintHeading1 ( "Global bundle adjustment" );
    if ( options.ba_global_use_pba && num_reg_images >= kMinNumRegImages &&
            ParallelBundleAdjuster::IsSupported ( custom_options,
                    mapper->GetReconstruction() ) ) {
        mapper->AdjustParallelGlobalBundle (
            custom_options, options.ParallelGlobalBundleAdjustment() );
    } else {
        mapper->AdjustGlobalBundle ( custom_options );
    }
}

void IterativeLocalRefinement ( const IncrementalMapperOptions& options,
                                const image_t image_id,
                                IncrementalMapper* mapper )
{
    auto ba_options = options.LocalBundleAdjustment();
    for ( int i = 0; i < options.ba_local_max_refinements; ++i ) {
        const auto report = mapper->AdjustLocalBundle (
                                options.Mapper(), ba_options, options.Triangulation(), image_id,
                                mapper->GetModifiedPoints3D() );
        std::cout << "  => Merged observations: " << report.num_merged_observations
                  << std::endl;
        std::cout << "  => Completed observations: "
                  << report.num_completed_observations << std::endl;
        std::cout << "  => Filtered observations: "
                  << report.num_filtered_observations << std::endl;
        const double changed =
            ( report.num_merged_observations + report.num_completed_observations +
              report.num_filtered_observations ) /
            static_cast<double> ( report.num_adjusted_observations );
        std::cout << StringPrintf ( "  => Changed observations: %.6f", changed )
                  << std::endl;
        if ( changed < options.ba_local_max_refinement_change ) {
            break;
        }
        // Only use robust cost function for first iteration.
        ba_options.loss_function_type =
            BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
    }
    mapper->ClearModifiedPoints3D();
}

void IterativeGlobalRefinement ( const IncrementalMapperOptions& options,
                                 IncrementalMapper* mapper,
                                 const std::function<void()>& update_func)
{
    PrintHeading1 ( "Retriangulation" );
    CompleteAndMergeTracks ( options, mapper );
    std::cout << "  => Retriangulated observations: "
              << mapper->Retriangulate ( options.Triangulation() ) << std::endl;

    for ( int i = 0; i < options.ba_global_max_refinements; ++i ) {
        const size_t num_observations =
            mapper->GetReconstruction().ComputeNumObservations();
        size_t num_changed_observations = 0;
        AdjustGlobalBundle ( options, mapper );
        num_changed_observations += CompleteAndMergeTracks ( options, mapper );
        num_changed_observations += FilterPoints ( options, mapper );
        const double changed =
            static_cast<double> ( num_changed_observations ) / num_observations;
        std::cout << StringPrintf ( "  => Changed observations: %.6f", changed )
                  << std::endl;

        update_func();

        if ( changed < options.ba_global_max_refinement_change ) {
            break;
        }
    }

    FilterImages ( options, mapper );
}

void ExtractColors ( const std::string& image_path, const image_t image_id,
                     Reconstruction* reconstruction )
{
    if ( !reconstruction->ExtractColorsForImage ( image_id, image_path ) ) {
        std::cout << StringPrintf ( "WARNING: Could not read image %s at path %s.",
                                    reconstruction->Image ( image_id ).Name().c_str(),
                                    image_path.c_str() )
                  << std::endl;
    }
}

void WriteSnapshot ( const Reconstruction& reconstruction,
                     const std::string& snapshot_path )
{
    PrintHeading1 ( "Creating snapshot" );
    // Get the current timestamp in milliseconds.
    const size_t timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds> (
            std::chrono::high_resolution_clock::now().time_since_epoch() )
        .count();
    // Write reconstruction to unique path with current timestamp.
    const std::string path =
        JoinPaths ( snapshot_path, StringPrintf ( "%010d", timestamp ) );
    CreateDirIfNotExists ( path );
    std::cout << "  => Writing to " << path << std::endl;
    reconstruction.Write ( path );
}

}  // namespace

size_t FilterPoints ( const IncrementalMapperOptions& options,
                      IncrementalMapper* mapper )
{
    const size_t num_filtered_observations =
        mapper->FilterPoints ( options.Mapper() );
    std::cout << "  => Filtered observations: " << num_filtered_observations
              << std::endl;
    return num_filtered_observations;
}

size_t FilterImages ( const IncrementalMapperOptions& options,
                      IncrementalMapper* mapper )
{
    const size_t num_filtered_images = mapper->FilterImages ( options.Mapper() );
    std::cout << "  => Filtered images: " << num_filtered_images << std::endl;
    return num_filtered_images;
}

size_t CompleteAndMergeTracks ( const IncrementalMapperOptions& options,
                                IncrementalMapper* mapper )
{
    const size_t num_completed_observations =
        mapper->CompleteTracks ( options.Triangulation() );
    std::cout << "  => Merged observations: " << num_completed_observations
              << std::endl;
    const size_t num_merged_observations =
        mapper->MergeTracks ( options.Triangulation() );
    std::cout << "  => Completed observations: " << num_merged_observations
              << std::endl;
    return num_completed_observations + num_merged_observations;
}

void SegmentImageNames ( const EIGEN_STL_UMAP ( image_t, class Image ) & image_map,
                         const size_t segment_size, const size_t segment_shared_size,
                         EIGEN_STL_UMAP ( size_t, std::set<std::string> ) * segment_image_names )
{
    PrintHeading1 ( "Segmentation is enabled" );
    // Collect all image base names and sort them.
    std::set<std::string> image_base_names;
    EIGEN_STL_UMAP ( std::string, std::vector<std::string> ) image_base_name_map;
    for ( const auto& image : image_map ) {
        const std::string image_name = image.second.Name();
        const std::string image_base_name = GetPathBaseName ( image_name );
        image_base_name_map[image_base_name].emplace_back ( image_name );
        image_base_names.insert ( image_base_name );
    }
    // Segment image names into different sets.
    size_t image_count = 0;
    size_t segment_num = ( size_t ) std::round ( image_base_names.size() / ( double ) segment_size );

    for ( const auto& image_base_name : image_base_names ) {
        size_t segment_idx = image_count / segment_size;
        segment_idx = std::min ( segment_idx, segment_num-1 );
        // Add to current segment.
        ( *segment_image_names ) [segment_idx].insert ( image_base_name_map[image_base_name].begin(),
                image_base_name_map[image_base_name].end() );
        // Add to previous segment.
        if ( segment_idx > 0
                && image_count - segment_idx*segment_size < segment_shared_size ) {
            ( *segment_image_names ) [segment_idx-1].insert ( image_base_name_map[image_base_name].begin(),
                    image_base_name_map[image_base_name].end() );
        }
        // Add to next segment.
        if ( segment_idx < segment_num-1
                && ( segment_idx+1 ) *segment_size - image_count < segment_shared_size ) {
            ( *segment_image_names ) [segment_idx+1].insert ( image_base_name_map[image_base_name].begin(),
                    image_base_name_map[image_base_name].end() );
        }
        image_count ++;
    }

    std::cout << StringPrintf ( "%d segments are collected:", segment_num ) << std::endl;
}

bool RecursiveSegmentMerging(const size_t idx1, const size_t idx2,
                             const IncrementalMapperOptions& options,
                             const std::string& database_path,
                             const std::function<void()>& func,
                             ReconstructionManager* reconstruction_manager) {
    CHECK_NOTNULL(reconstruction_manager);
    size_t segment_num = reconstruction_manager->Size();

    CHECK_LT(idx1, segment_num);
    CHECK_LT(idx2, segment_num);
    CHECK_GE(idx1, 0);
    CHECK_GE(idx2, 0);
    CHECK_LE(idx1, idx2);

    if(idx1 == idx2) {
        return true;
    } else {
        bool merged1 = false;
        bool merged2 = false;
        size_t segment_idx1 = idx1;
        size_t segment_idx2 = idx2;

        if(idx1 == idx2-1) {
            merged1 = merged2 = true;
            PrintHeading2(StringPrintf("Merging [%d] to [%d]", idx2, idx1));
        } else {
            size_t mid = (idx1 + idx2) / 2;
            merged1 = RecursiveSegmentMerging(idx1, mid, options, database_path, func,
                                              reconstruction_manager);
            if(mid+1 == idx2) {
                merged2 = true;
                PrintHeading2(StringPrintf("Merging [%d] to [%d-%d]", idx2, idx1, mid));
            } else {
                merged2 = RecursiveSegmentMerging(mid+1, idx2, options, database_path, func,
                                                  reconstruction_manager);
                PrintHeading2(StringPrintf("Merging [%d-%d] to [%d-%d]", mid+1, idx2, idx1, mid));
                segment_idx2 = mid+1;
            }
        }

        if(merged1 && merged2) {
            Reconstruction& recon1 = reconstruction_manager->Get(segment_idx1);
            Reconstruction& recon2 = reconstruction_manager->Get(segment_idx2);

            if(recon1.Merge(recon2, options.segment_merge_max_reproj_error)) {
                std::cout << "  => Succeed" << std::endl;
                std::cout << "  => Merged images: " << recon1.NumImages() << std::endl;
                std::cout << "  => Merged points: " << recon1.NumPoints3D() << std::endl;

                func();

                if(options.segment_merge_final_ba) {
                    // Collect image names from reconstruction.
                    std::set<std::string> image_names;
                    const EIGEN_STL_UMAP(image_t, class Image) images = recon1.Images();
                    for(const auto& image : images) {
                        image_names.insert(image.second.Name());
                    }

                    // Prepare for BA
                    DatabaseCache database_cache;
                    Database database ( database_path );
                    const size_t min_num_matches = static_cast<size_t> ( options.min_num_matches );
                    database_cache.Load ( database, min_num_matches, options.ignore_watermarks,
                                          image_names);
                    database.Close();

                    // Perform BA
                    IncrementalMapper mapper (&database_cache);
                    mapper.BeginReconstruction(&recon1);
                    IncrementalMapperOptions overriden_option (options);
                    overriden_option.ba_refine_extra_params = true;
                    overriden_option.ba_refine_focal_length = true;
                    overriden_option.ba_refine_principal_point = true;
                    IterativeGlobalRefinement(overriden_option, &mapper, func);
                    const bool kDiscardReconstruction = false;
                    mapper.EndReconstruction(kDiscardReconstruction);
                }

                if(!options.segment_merge_output_path.empty()) {
                    std::string path = options.segment_merge_output_path;
                    if(options.segment_merge_final_ba) {
                        path = JoinPaths(path, "merge-ba-steps");
                        CreateDirIfNotExists(path);
                    } else {
                        path = JoinPaths(path, "merge-steps");
                        CreateDirIfNotExists(path);
                    }
                    path = JoinPaths(path, StringPrintf("%d-%d", idx1, idx2));
                    CreateDirIfNotExists(path);
                    recon1.Write(path);
                }

                return true;
            } else {
                std::cout << "  => Failed" << std::endl;
                return false;
            }
        } else {
            return false;
        }
    }
}

bool MergeSegmentsBottomUp(const IncrementalMapperOptions& options,
                           const std::string& database_path,
                           const std::function<void()>& func,
                           ReconstructionManager* reconstruction_manager) {
    CHECK_NOTNULL(reconstruction_manager);

    bool merge_failed = false;
    size_t segment_num = reconstruction_manager->Size();
    std::vector<size_t> segment_merge_size (segment_num, 1);

    while(reconstruction_manager->Size() > 1 && !merge_failed) {
        segment_num = reconstruction_manager->Size();
        size_t segment_merge_processed_size = 0;

        // Merge odd index segment into even index segment.
        for(size_t idx=0; idx+1<segment_num; idx+=2) {
            size_t current_segment_merge_size = segment_merge_size[idx] + segment_merge_size[idx+1];
            if(segment_merge_size[idx] == 1 && segment_merge_size[idx+1] == 1) {
                PrintHeading2(StringPrintf("Merging [%d] to [%d]", idx+1, idx));
            } else if(segment_merge_size[idx+1] == 1) {
                PrintHeading2(StringPrintf("Merging [%d] to [%d-%d]",
                                           segment_merge_processed_size+segment_merge_size[idx],
                                           segment_merge_processed_size,
                                           segment_merge_processed_size+segment_merge_size[idx]-1));
            } else {
                PrintHeading2(StringPrintf("Merging [%d-%d] to [%d-%d]",
                                           segment_merge_processed_size+segment_merge_size[idx],
                                           segment_merge_processed_size+current_segment_merge_size-1,
                                           segment_merge_processed_size,
                                           segment_merge_processed_size+segment_merge_size[idx]-1));
            }
            segment_merge_size[idx] = current_segment_merge_size;

            Reconstruction& recon1 = reconstruction_manager->Get(idx);
            Reconstruction& recon2 = reconstruction_manager->Get(idx+1);

            if(recon1.Merge(recon2, options.segment_merge_max_reproj_error)) {
                std::cout << "  => Succeed" << std::endl;
                std::cout << "  => Merged images: " << recon1.NumImages() << std::endl;
                std::cout << "  => Merged points: " << recon1.NumPoints3D() << std::endl;

                func();

                // Perform BA if selected.
                if(options.segment_merge_final_ba) {
                    // Collect image names from reconstruction.
                    std::set<std::string> image_names;
                    const EIGEN_STL_UMAP(image_t, class Image) images = recon1.Images();
                    for(const auto& image : images) {
                        image_names.insert(image.second.Name());
                    }

                    // Prepare for BA
                    DatabaseCache database_cache;
                    Database database ( database_path );
                    const size_t min_num_matches = static_cast<size_t> ( options.min_num_matches );
                    database_cache.Load ( database, min_num_matches, options.ignore_watermarks,
                                          image_names);
                    database.Close();

                    // Perform BA
                    IncrementalMapper mapper (&database_cache);
                    mapper.BeginReconstruction(&recon1);
                    IterativeGlobalRefinement(options, &mapper, func);
                    const bool kDiscardReconstruction = false;
                    mapper.EndReconstruction(kDiscardReconstruction);
                }

                // Save current merge stage.
                if(!options.segment_merge_output_path.empty()) {
                    std::string path = options.segment_merge_output_path;
                    if(options.segment_merge_final_ba) {
                        path = JoinPaths(path, "merge-ba-steps");
                        CreateDirIfNotExists(path);
                    } else {
                        path = JoinPaths(path, "merge-steps");
                        CreateDirIfNotExists(path);
                    }
                    path = JoinPaths(path, StringPrintf("%d-%d", segment_merge_processed_size,
                                                        segment_merge_processed_size+current_segment_merge_size));
                    CreateDirIfNotExists(path);
                    recon1.Write(path);
                }
                segment_merge_processed_size += current_segment_merge_size;
            } else {
                std::cout << "  => Failed" << std::endl;
                merge_failed = true;
                break;
            }
        }
        // Remove odd index segment from reconstruction manager to save memory.
        size_t delete_idx = 1;
        while(delete_idx < reconstruction_manager->Size()){
            reconstruction_manager->Delete(delete_idx);
            delete_idx ++;
        }
        
        // Update segment merge size.
        for(size_t idx=0; idx<segment_merge_size.size(); idx+=2) {
            segment_merge_size[idx/2] = segment_merge_size[idx];
        }
        segment_merge_size.resize(reconstruction_manager->Size());
    }

    return !merge_failed;
}

IncrementalMapper::Options IncrementalMapperOptions::Mapper() const
{
    IncrementalMapper::Options options = mapper;
    options.abs_pose_refine_focal_length = ba_refine_focal_length;
    options.abs_pose_refine_extra_params = ba_refine_extra_params;
    options.min_focal_length_ratio = min_focal_length_ratio;
    options.max_focal_length_ratio = max_focal_length_ratio;
    options.max_principal_point_error_ratio = max_principal_point_error_ratio;
    options.max_extra_param = max_extra_param;
    options.num_threads = num_threads;
    options.local_ba_num_images = ba_local_num_images;
    return options;
}

IncrementalTriangulator::Options IncrementalMapperOptions::Triangulation()
const
{
    IncrementalTriangulator::Options options = triangulation;
    options.min_focal_length_ratio = min_focal_length_ratio;
    options.max_focal_length_ratio = max_focal_length_ratio;
    options.max_principal_point_error_ratio = max_principal_point_error_ratio;
    options.max_extra_param = max_extra_param;
    return options;
}

BundleAdjustmentOptions IncrementalMapperOptions::LocalBundleAdjustment()
const
{
    BundleAdjustmentOptions options;
    options.solver_options.function_tolerance = 0.0;
    options.solver_options.gradient_tolerance = 10.0;
    options.solver_options.parameter_tolerance = 0.0;
    options.solver_options.max_num_iterations = ba_local_max_num_iterations;
    options.solver_options.max_linear_solver_iterations = 100;
    options.solver_options.minimizer_progress_to_stdout = false;
    options.solver_options.num_threads = num_threads;
#if CERES_VERSION_MAJOR < 2
    options.solver_options.num_linear_solver_threads = num_threads;
#endif  // CERES_VERSION_MAJOR
    options.print_summary = true;
    options.refine_focal_length = ba_refine_focal_length;
    options.refine_principal_point = ba_refine_principal_point;
    options.refine_extra_params = ba_refine_extra_params;
    options.loss_function_scale = 1.0;
    options.loss_function_type =
        BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
    return options;
}

BundleAdjustmentOptions IncrementalMapperOptions::GlobalBundleAdjustment()
const
{
    BundleAdjustmentOptions options;
    options.solver_options.function_tolerance = 0.0;
    options.solver_options.gradient_tolerance = 1.0;
    options.solver_options.parameter_tolerance = 0.0;
    options.solver_options.max_num_iterations = ba_global_max_num_iterations;
    options.solver_options.max_linear_solver_iterations = 100;
    options.solver_options.minimizer_progress_to_stdout = true;
    options.solver_options.num_threads = num_threads;
#if CERES_VERSION_MAJOR < 2
    options.solver_options.num_linear_solver_threads = num_threads;
#endif  // CERES_VERSION_MAJOR
    options.print_summary = true;
    options.refine_focal_length = ba_refine_focal_length;
    options.refine_principal_point = ba_refine_principal_point;
    options.refine_extra_params = ba_refine_extra_params;
    options.loss_function_type =
        BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
    return options;
}

ParallelBundleAdjuster::Options
IncrementalMapperOptions::ParallelGlobalBundleAdjustment() const
{
    ParallelBundleAdjuster::Options options;
    options.max_num_iterations = ba_global_max_num_iterations;
    options.print_summary = true;
    options.gpu_index = ba_global_pba_gpu_index;
    options.num_threads = num_threads;
    return options;
}

bool IncrementalMapperOptions::Check() const
{
    CHECK_OPTION_GT ( min_num_matches, 0 );
    CHECK_OPTION_GT ( max_num_models, 0 );
    CHECK_OPTION_GT ( max_model_overlap, 0 );
    CHECK_OPTION_GE ( min_model_size, 0 );
    CHECK_OPTION_GT ( init_num_trials, 0 );
    CHECK_OPTION_GT ( min_focal_length_ratio, 0 );
    CHECK_OPTION_GT ( max_focal_length_ratio, 0 );
    CHECK_OPTION_GE ( max_extra_param, 0 );
    CHECK_OPTION_GE ( ba_local_num_images, 2 );
    CHECK_OPTION_GE ( ba_local_max_num_iterations, 0 );
    CHECK_OPTION_GT ( ba_global_images_ratio, 1.0 );
    CHECK_OPTION_GT ( ba_global_points_ratio, 1.0 );
    CHECK_OPTION_GT ( ba_global_images_freq, 0 );
    CHECK_OPTION_GT ( ba_global_points_freq, 0 );
    CHECK_OPTION_GT ( ba_global_max_num_iterations, 0 );
    CHECK_OPTION_GT ( ba_local_max_refinements, 0 );
    CHECK_OPTION_GE ( ba_local_max_refinement_change, 0 );
    CHECK_OPTION_GT ( ba_global_max_refinements, 0 );
    CHECK_OPTION_GE ( ba_global_max_refinement_change, 0 );
    CHECK_OPTION_GE ( snapshot_images_freq, 0 );
    CHECK_OPTION_GE ( segment_size, 0 );
    CHECK_OPTION_GE ( segment_shared_size, 0 );
    CHECK_OPTION_LE ( segment_shared_size, segment_size );
    CHECK_OPTION_GT ( segment_merge_max_reproj_error, 0 );
    CHECK_OPTION ( Mapper().Check() );
    CHECK_OPTION ( Triangulation().Check() );
    return true;
}

IncrementalMapperController::IncrementalMapperController (
    const IncrementalMapperOptions* options, const std::string& image_path,
    const std::string& database_path,
    ReconstructionManager* reconstruction_manager )
    : options_ ( options ),
      image_path_ ( image_path ),
      database_path_ ( database_path ),
      reconstruction_manager_ ( reconstruction_manager )
{
    CHECK ( options_->Check() );
    RegisterCallback ( INITIAL_IMAGE_PAIR_REG_CALLBACK );
    RegisterCallback ( NEXT_IMAGE_REG_CALLBACK );
    RegisterCallback ( LAST_IMAGE_REG_CALLBACK );
}

void IncrementalMapperController::Run()
{
    if ( !LoadDatabase() ) {
        return;
    }

    IncrementalMapper::Options init_mapper_options = options_->Mapper();
    Reconstruct ( init_mapper_options );

    const size_t kNumInitRelaxations = 2;
    for ( size_t i = 0; i < kNumInitRelaxations; ++i ) {
        if ( reconstruction_manager_->Size() > 0 || IsStopped() ) {
            break;
        }

        std::cout << "  => Relaxing the initialization constraints." << std::endl;
        init_mapper_options.init_min_num_inliers /= 2;
        Reconstruct ( init_mapper_options );

        if ( reconstruction_manager_->Size() > 0 || IsStopped() ) {
            break;
        }

        std::cout << "  => Relaxing the initialization constraints." << std::endl;
        init_mapper_options.init_min_tri_angle /= 2;
        Reconstruct ( init_mapper_options );
    }

    std::cout << std::endl;
    GetTimer().PrintMinutes();
}

bool IncrementalMapperController::LoadDatabase()
{
    PrintHeading1 ( "Loading database to cache" );

    Database database ( database_path_ );
    Timer timer;
    timer.Start();

    const size_t min_num_matches = static_cast<size_t> ( options_->min_num_matches );
    database_cache_.Load ( database, min_num_matches, options_->ignore_watermarks,
                           options_->image_names );

    database.Close();

    std::cout << std::endl;
    timer.PrintMinutes();

    std::cout << std::endl;

    if ( database_cache_.NumImages() == 0 ) {
        std::cout << "WARNING: No images with matches found in the database."
                  << std::endl
                  << std::endl;
        return false;
    }

    return true;
}

void IncrementalMapperController::Reconstruct (
    const IncrementalMapper::Options& init_mapper_options )
{
    // If segment reconstruction is not enabled, perform segment reconstruction
    // with the whole database cache.
    if ( options_->segment_size == 0 ) {
        ReconstructSegment ( init_mapper_options, database_cache_ );
    } else {
        bool initial_reconstruction_given = reconstruction_manager_->Size() > 0;
        EIGEN_STL_UMAP ( size_t, std::set<std::string> ) segment_image_names;
        SegmentImageNames ( database_cache_.Images(), options_->segment_size,
                            options_->segment_shared_size, &segment_image_names );

        if(initial_reconstruction_given) {
            PrintHeading1("Reconstruction skipped");
        } else {
            size_t segment_idx = 0;
            while ( segment_idx < segment_image_names.size() ) {
                PrintHeading1 (
                    StringPrintf ( "Reconstruct Segment %d (%d images)", segment_idx+1,
                                   segment_image_names[segment_idx].size() ) );

                // Prepare a new database cache for current segment.
                DatabaseCache segment_database_cache;
                Database database ( database_path_ );
                const size_t min_num_matches = static_cast<size_t> ( options_->min_num_matches );
                segment_database_cache.Load ( database, min_num_matches, options_->ignore_watermarks,
                                              segment_image_names[segment_idx] );
                database.Close();

                ReconstructSegment ( init_mapper_options, segment_database_cache );

                if(IsStopped()) {
                    return;
                }

                // Save segment reconstruction.
                std::string path = JoinPaths ( options_->segment_merge_output_path, "segments");
                CreateDirIfNotExists(path);
                path = JoinPaths(path, std::to_string(segment_idx) );
                CreateDirIfNotExists ( path );
                std::cout << "  => Saving segment " << segment_idx << std::endl;
                reconstruction_manager_->Get(reconstruction_manager_->Size()-1).Write ( path );

                segment_idx ++;
            }
        }

        // Perform merge and BA
        if ( reconstruction_manager_->Size() > 1 ) {
            PrintHeading1 ( "Merge segment reconstructions" );
//             bool merge_succeed = RecursiveSegmentMerging(0, reconstruction_manager_->Size()-1,
//             *options_, database_path_, [this]() {
//                 Callback( LAST_IMAGE_REG_CALLBACK );
//             }, reconstruction_manager_ );
            bool merge_succeed = MergeSegmentsBottomUp(*options_, database_path_, [this]() {
                Callback( LAST_IMAGE_REG_CALLBACK );
            }, reconstruction_manager_ );
            if(!merge_succeed) {
                PrintHeading1("Something wrong happened during merging.");
                return;
            }
        }

        // Save merged reconstruction after bundle adjustment if output path is given.
        if ( !options_->segment_merge_output_path.empty() && options_->segment_merge_final_ba ) {
            std::string path = JoinPaths ( options_->segment_merge_output_path, "merged-ba" );
            CreateDirIfNotExists ( path );
            std::cout << "  => Saving merged model to " << path << std::endl;
            reconstruction_manager_->Get(0).Write ( path );
        }
    }
}

void IncrementalMapperController::ReconstructSegment (
    const IncrementalMapper::Options& init_mapper_options, DatabaseCache& database_cache )
{
    const bool kDiscardReconstruction = true;
    const bool kSegmentReconstructionEnabled = options_->segment_size > 0;

    //////////////////////////////////////////////////////////////////////////////
    // Main loop
    //////////////////////////////////////////////////////////////////////////////

    IncrementalMapper mapper ( &database_cache );

    // Is there a sub-model before we start the reconstruction? I.e. the user
    // has imported an existing reconstruction.
    const bool initial_reconstruction_given = reconstruction_manager_->Size() > 0;
    if ( !kSegmentReconstructionEnabled ) {
        CHECK_LE ( reconstruction_manager_->Size(), 1 )
                << "Can only resume from a single reconstruction, but multiple are given.";
    }

    for ( int num_trials = 0; num_trials < options_->init_num_trials;
            ++num_trials ) {
        BlockIfPaused();
        if ( IsStopped() ) {
            break;
        }

        size_t reconstruction_idx;
        if ( !initial_reconstruction_given || num_trials > 0 || kSegmentReconstructionEnabled ) {
            reconstruction_idx = reconstruction_manager_->Add();
        } else {
            reconstruction_idx = 0;
        }

        Reconstruction& reconstruction =
            reconstruction_manager_->Get ( reconstruction_idx );

        mapper.BeginReconstruction ( &reconstruction );

        if ( kSegmentReconstructionEnabled && reconstruction_manager_->Size() > 1 ) {
            //mapper.CopyCamerasFrom ( reconstruction_manager_->Get ( 0 ) );
        }

        ////////////////////////////////////////////////////////////////////////////
        // Register initial pair
        ////////////////////////////////////////////////////////////////////////////

        if ( reconstruction.NumRegImages() == 0 ) {
            image_t image_id1 = static_cast<image_t> ( options_->init_image_id1 );
            image_t image_id2 = static_cast<image_t> ( options_->init_image_id2 );

            // Try to find good initial pair.
            if ( options_->init_image_id1 == -1 || options_->init_image_id2 == -1 ) {
                const bool find_init_success = mapper.FindInitialImagePair (
                                                   init_mapper_options, &image_id1, &image_id2 );
                if ( !find_init_success ) {
                    std::cout << "  => No good initial image pair found." << std::endl;
                    mapper.EndReconstruction ( kDiscardReconstruction );
                    reconstruction_manager_->Delete ( reconstruction_idx );
                    break;
                }
            } else {
                if ( !reconstruction.ExistsImage ( image_id1 ) ||
                        !reconstruction.ExistsImage ( image_id2 ) ) {
                    std::cout << StringPrintf (
                                  "  => Initial image pair #%d and #%d do not exist.",
                                  image_id1, image_id2 )
                              << std::endl;
                    mapper.EndReconstruction ( kDiscardReconstruction );
                    reconstruction_manager_->Delete ( reconstruction_idx );
                    return;
                }
            }

            PrintHeading1 ( StringPrintf ( "Initializing with image pair #%d and #%d",
                                           image_id1, image_id2 ) );
            const bool reg_init_success = mapper.RegisterInitialImagePair (
                                              init_mapper_options, image_id1, image_id2 );
            if ( !reg_init_success ) {
                std::cout << "  => Initialization failed - possible solutions:"
                          << std::endl
                          << "     - try to relax the initialization constraints"
                          << std::endl
                          << "     - manually select an initial image pair"
                          << std::endl;
                mapper.EndReconstruction ( kDiscardReconstruction );
                reconstruction_manager_->Delete ( reconstruction_idx );
                break;
            }

            AdjustGlobalBundle ( *options_, &mapper );
            FilterPoints ( *options_, &mapper );
            FilterImages ( *options_, &mapper );

            // Initial image pair failed to register.
            if ( reconstruction.NumRegImages() == 0 ||
                    reconstruction.NumPoints3D() == 0 ) {
                std::cout << "  => Initial image pair failed to register, try again." << std::endl;
                mapper.EndReconstruction ( kDiscardReconstruction );
                reconstruction_manager_->Delete ( reconstruction_idx );
                // If both initial images are manually specified, there is no need for
                // further initialization trials.
                if ( options_->init_image_id1 != -1 && options_->init_image_id2 != -1 ) {
                    break;
                } else {
                    continue;
                }
            }

            if ( options_->extract_colors ) {
                ExtractColors ( image_path_, image_id1, &reconstruction );
            }
        }

        Callback ( INITIAL_IMAGE_PAIR_REG_CALLBACK );

        ////////////////////////////////////////////////////////////////////////////
        // Incremental mapping
        ////////////////////////////////////////////////////////////////////////////

        size_t snapshot_prev_num_reg_images = reconstruction.NumRegImages();
        size_t ba_prev_num_reg_images = reconstruction.NumRegImages();
        size_t ba_prev_num_points = reconstruction.NumPoints3D();

        bool reg_next_success = true;
        bool prev_reg_next_success = true;
        while ( reg_next_success ) {
            BlockIfPaused();
            if ( IsStopped() ) {
                break;
            }

            reg_next_success = false;

            const std::vector<image_t> next_images =
                mapper.FindNextImages ( options_->Mapper() );

            if ( next_images.empty() ) {
                std::cout << "  => No more next images to process." << std::endl;
                break;
            }

            for ( size_t reg_trial = 0; reg_trial < next_images.size(); ++reg_trial ) {
                const image_t next_image_id = next_images[reg_trial];
                const Image& next_image = reconstruction.Image ( next_image_id );

                PrintHeading1 ( StringPrintf ( "Registering image #%d (%d)", next_image_id,
                                               reconstruction.NumRegImages() + 1 ) );

                std::cout << StringPrintf ( "  => Image sees %d / %d points",
                                            next_image.NumVisiblePoints3D(),
                                            next_image.NumObservations() )
                          << std::endl;

                reg_next_success =
                    mapper.RegisterNextImage ( options_->Mapper(), next_image_id );

                if ( reg_next_success ) {
                    TriangulateImage ( *options_, next_image, &mapper );
                    IterativeLocalRefinement ( *options_, next_image_id, &mapper );

                    if ( reconstruction.NumRegImages() >=
                            options_->ba_global_images_ratio * ba_prev_num_reg_images ||
                            reconstruction.NumRegImages() >=
                            options_->ba_global_images_freq + ba_prev_num_reg_images ||
                            reconstruction.NumPoints3D() >=
                            options_->ba_global_points_ratio * ba_prev_num_points ||
                            reconstruction.NumPoints3D() >=
                            options_->ba_global_points_freq + ba_prev_num_points ) {
                        std::cout << "  => Need a temporal global bundle adjustment"
                                  << std::endl;
                        IterativeGlobalRefinement ( *options_, &mapper,
                        [this]() {
                            Callback(LAST_IMAGE_REG_CALLBACK);
                        });
                        ba_prev_num_points = reconstruction.NumPoints3D();
                        ba_prev_num_reg_images = reconstruction.NumRegImages();
                    }

                    if ( options_->extract_colors ) {
                        ExtractColors ( image_path_, next_image_id, &reconstruction );
                    }

                    if ( options_->snapshot_images_freq > 0 &&
                            reconstruction.NumRegImages() >=
                            options_->snapshot_images_freq +
                            snapshot_prev_num_reg_images ) {
                        snapshot_prev_num_reg_images = reconstruction.NumRegImages();
                        WriteSnapshot ( reconstruction, options_->snapshot_path );
                    }

                    Callback ( NEXT_IMAGE_REG_CALLBACK );

                    break;
                } else {
                    std::cout << "  => Could not register, trying another image."
                              << std::endl;

                    // If initial pair fails to continue for some time,
                    // abort and try different initial pair.
                    const size_t kMinNumInitialRegTrials = 30;
                    if ( reg_trial >= kMinNumInitialRegTrials &&
                            reconstruction.NumRegImages() <
                            static_cast<size_t> ( options_->min_model_size ) ) {
                        break;
                    }
                }
            }

            const size_t max_model_overlap =
                static_cast<size_t> ( options_->max_model_overlap );
            if ( mapper.NumSharedRegImages() >= max_model_overlap ) {
                std::cout << "  => Shared registered images exceed threshold." << std::endl;
                break;
            }

            // If no image could be registered, try a single final global iterative
            // bundle adjustment and try again to register one image. If this fails
            // once, then exit the incremental mapping.
            if ( !reg_next_success && prev_reg_next_success ) {
                reg_next_success = true;
                prev_reg_next_success = false;
                std::cout << "  => No image could be registered, a finalized global bundle adjustment"
                          << std::endl;
                IterativeGlobalRefinement ( *options_, &mapper,
                [this]() {
                    Callback(LAST_IMAGE_REG_CALLBACK);
                });
            } else {
                prev_reg_next_success = reg_next_success;
            }
        }

        if ( IsStopped() ) {
            const bool kDiscardReconstruction = false;
            mapper.EndReconstruction ( kDiscardReconstruction );
            break;
        }

        // Only run final global BA, if last incremental BA was not global.
        if ( reconstruction.NumRegImages() >= 2 &&
                reconstruction.NumRegImages() != ba_prev_num_reg_images &&
                reconstruction.NumPoints3D() != ba_prev_num_points ) {
            IterativeGlobalRefinement ( *options_, &mapper,
            [this]() {
                Callback(LAST_IMAGE_REG_CALLBACK);
            } );
        }

        // If the total number of images is small then do not enforce the minimum
        // model size so that we can reconstruct small image collections.
        const size_t min_model_size =
            std::min ( database_cache.NumImages(),
                       static_cast<size_t> ( options_->min_model_size ) );
        if ( ( options_->multiple_models &&
                reconstruction.NumRegImages() < min_model_size ) ||
                reconstruction.NumRegImages() == 0 ||
                ( kSegmentReconstructionEnabled &&
                  reconstruction.NumRegImages() < 0.6 * options_->segment_size ) ) {
            std::cout << "  => Current reconstruction model is too small. Deleting" << std::endl;
            mapper.EndReconstruction ( kDiscardReconstruction );
            reconstruction_manager_->Delete ( reconstruction_idx );
        } else {
            const bool kDiscardReconstruction = false;
            mapper.EndReconstruction ( kDiscardReconstruction );
        }

        Callback ( LAST_IMAGE_REG_CALLBACK );

        const size_t max_num_models = static_cast<size_t> ( options_->max_num_models );
        if ( ( !kSegmentReconstructionEnabled && initial_reconstruction_given ) ||
                !options_->multiple_models || reconstruction_manager_->Size() >= max_num_models ||
                mapper.NumTotalRegImages() >= database_cache.NumImages() - 1 ) {
            std::cout << "  => Conditions not to further reconstruct are met." << std::endl;
            break;
        }
    }
}

}  // namespace colmap






