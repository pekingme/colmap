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

#ifndef COLMAP_SRC_CONTROLLERS_INCREMENTAL_MAPPER_H_
#define COLMAP_SRC_CONTROLLERS_INCREMENTAL_MAPPER_H_

#include "base/reconstruction_manager.h"
#include "sfm/incremental_mapper.h"
#include "util/threading.h"

namespace colmap
{

struct IncrementalMapperOptions {
public:
    // The minimum number of matches for inlier matches to be considered.
    int min_num_matches = 15;

    // Whether to ignore the inlier matches of watermark image pairs.
    bool ignore_watermarks = false;

    // Whether to reconstruct multiple sub-models.
    bool multiple_models = true;

    // The number of sub-models to reconstruct.
    int max_num_models = 50;

    // The maximum number of overlapping images between sub-models. If the
    // current sub-models shares more than this number of images with another
    // model, then the reconstruction is stopped.
    int max_model_overlap = 20;

    // The minimum number of registered images of a sub-model, otherwise the
    // sub-model is discarded.
    int min_model_size = 10;

    // The image identifiers used to initialize the reconstruction. Note that
    // only one or both image identifiers can be specified. In the former case,
    // the second image is automatically determined.
    int init_image_id1 = -1;
    int init_image_id2 = -1;

    // The number of trials to initialize the reconstruction.
    int init_num_trials = 200;

    // Whether to extract colors for reconstructed points.
    bool extract_colors = true;

    // The number of threads to use during reconstruction.
    int num_threads = -1;

    // Thresholds for filtering images with degenerate intrinsics.
    double min_focal_length_ratio = 0.1;
    double max_focal_length_ratio = 10.0;
    double max_principal_point_error_ratio = 0.05;
    double max_extra_param = 1.0;

    // Which intrinsic parameters to optimize during the reconstruction.
    bool ba_refine_focal_length = true;
    bool ba_refine_principal_point = false;
    bool ba_refine_extra_params = true;

    // The number of images to optimize in local bundle adjustment.
    int ba_local_num_images = 6;

    // The maximum number of local bundle adjustment iterations.
    int ba_local_max_num_iterations = 25;

    // Whether to use PBA in global bundle adjustment.
    bool ba_global_use_pba = true;

    // The GPU index for PBA bundle adjustment.
    int ba_global_pba_gpu_index = -1;

    // The growth rates after which to perform global bundle adjustment.
    double ba_global_images_ratio = 1.1;
    double ba_global_points_ratio = 1.1;
    int ba_global_images_freq = 500;
    int ba_global_points_freq = 250000;

    // The maximum number of global bundle adjustment iterations.
    int ba_global_max_num_iterations = 50;

    // The thresholds for iterative bundle adjustment refinements.
    int ba_local_max_refinements = 2;
    double ba_local_max_refinement_change = 0.001;
    int ba_global_max_refinements = 5;
    double ba_global_max_refinement_change = 0.0005;

    // Path to a folder with reconstruction snapshots during incremental
    // reconstruction. Snapshots will be saved according to the specified
    // frequency of registered images.
    std::string snapshot_path = "";
    int snapshot_images_freq = 0;

    // Which images to reconstruct. If no images are specified, all images will
    // be reconstructed by default.
    std::set<std::string> image_names;

    // The number of images (or snapshots) per block in segment reconstruction.
    // 0 for segment reconstruction not enabled.
    int segment_size = 0;

    // The number of images (or snapshots) per block shared with the neighbor
    // blocks in segment reconstruction.
    int segment_shared_size = 0;

    // The max reprojection error for model merging. See function
    // ComputeAlignmentBetweenReconstructions in SimilarityTransform.h
    double segment_merge_max_reproj_error = 64.0;

    // Path to a folder with merged segment reconstruction. If the path is empty,
    // merged segment reconstruction model will not be saved automatically.
    std::string segment_merge_output_path = "";

    // Whether to perform a global bundle adjustment in the merged model.
    bool segment_merge_final_ba = true;

    IncrementalMapper::Options Mapper() const;
    IncrementalTriangulator::Options Triangulation() const;
    BundleAdjustmentOptions LocalBundleAdjustment() const;
    BundleAdjustmentOptions GlobalBundleAdjustment() const;
    ParallelBundleAdjuster::Options ParallelGlobalBundleAdjustment() const;

    bool Check() const;

private:
    friend class OptionManager;
    friend class MapperGeneralOptionsWidget;
    friend class MapperTriangulationOptionsWidget;
    friend class MapperRegistrationOptionsWidget;
    friend class MapperInitializationOptionsWidget;
    friend class MapperBundleAdjustmentOptionsWidget;
    friend class MapperFilteringOptionsWidget;
    friend class ReconstructionOptionsWidget;
    IncrementalMapper::Options mapper;
    IncrementalTriangulator::Options triangulation;
};

// Class that controls the incremental mapping procedure by iteratively
// initializing reconstructions from the same scene graph.
class IncrementalMapperController : public Thread
{
public:
    enum {
        INITIAL_IMAGE_PAIR_REG_CALLBACK,
        NEXT_IMAGE_REG_CALLBACK,
        LAST_IMAGE_REG_CALLBACK,
    };

    IncrementalMapperController ( const IncrementalMapperOptions* options,
                                  const std::string& image_path,
                                  const std::string& database_path,
                                  ReconstructionManager* reconstruction_manager );

private:
    void Run();
    bool LoadDatabase();
    void Reconstruct ( const IncrementalMapper::Options& init_mapper_options );
    void ReconstructSegment ( const IncrementalMapper::Options& init_mapper_options,
                              DatabaseCache& database_cache );

    const IncrementalMapperOptions* options_;
    const std::string image_path_;
    const std::string database_path_;
    ReconstructionManager* reconstruction_manager_;
    DatabaseCache database_cache_;
};

// Globally filter points and images in mapper.
size_t FilterPoints ( const IncrementalMapperOptions& options,
                      IncrementalMapper* mapper );
size_t FilterImages ( const IncrementalMapperOptions& options,
                      IncrementalMapper* mapper );

// Globally complete and merge tracks in mapper.
size_t CompleteAndMergeTracks (
    const IncrementalMapperOptions& options,
    IncrementalMapper* mapper );

// Segment image names into groups for segment reconstruction.
void SegmentImageNames ( const EIGEN_STL_UMAP ( image_t, class Image ) & image_map,
                         const size_t segment_size, const size_t segment_shared_size,
                         EIGEN_STL_UMAP ( int, std::set<std::string> ) * segment_image_names );

// Merge reconstructions in post order traversal.
bool RecursiveSegmentMerging(const size_t idx1, const size_t idx2,
                             const IncrementalMapperOptions& options,
                             const std::string& database_path,
                             const std::function<void()>& func,
                             ReconstructionManager* reconstruction_manager);

// Merge segments in bottom up level order traversal.
bool MergeSegmentsBottomUp(const IncrementalMapperOptions& options, 
                           const std::string& database_path, 
                           const std::function<void()>& func, 
                           ReconstructionManager* reconstruction_manager);

}  // namespace colmap

#endif  // COLMAP_SRC_CONTROLLERS_INCREMENTAL_MAPPER_H_

