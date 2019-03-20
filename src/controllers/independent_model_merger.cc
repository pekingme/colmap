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

#include "independent_model_merger.h"

#include "base/database.h"
#include "base/database_cache.h"
#include "base/similarity_transform.h"
#include "feature/matching.h"
#include "controllers/incremental_mapper.h"
#include "controllers/bundle_adjustment.h"
#include "util/misc.h"

namespace colmap {

IndependentModelMergerController::IndependentModelMergerController(
    const Options& options, const OptionManager& option_manager)
    : options_(options),
      option_manager_(option_manager) {
    // Check option manager settings.
    CHECK(option_manager_.cross_group_matching);
    CHECK(option_manager_.vocab_tree_matching);
    CHECK(option_manager_.sift_matching);
    CHECK(option_manager_.mapper);
    if(options_.global_ba) {
        CHECK(option_manager_.bundle_adjustment);
    }

    // Override options.
    option_manager_.vocab_tree_matching->vocab_tree_path = options_.vocab_tree_path;
    option_manager_.sift_matching->use_gpu = options_.use_gpu;
    option_manager_.sift_matching->gpu_index = options_.gpu_index;

    // Check all required files and direcotries.
    CHECK(ExistsFile(options_.vocab_tree_path));

    CHECK(ExistsDir(options_.workspace_path1));
    CHECK(ExistsDir(options_.workspace_path2));
    CHECK(ExistsDir(options_.workspace_path));

    // Image directoris. Assume directory name "images" is used.
    CHECK(ExistsDir(JoinPaths(options_.workspace_path1, "images")));
    CHECK(ExistsDir(JoinPaths(options_.workspace_path2, "images")));
    CreateDirIfNotExists(JoinPaths(options_.workspace_path, "images"));

    // Database files. Assume file name "database.db" is used.
    CHECK(ExistsFile(JoinPaths(options_.workspace_path1, "database.db")));
    CHECK(ExistsFile(JoinPaths(options_.workspace_path2, "database.db")));
    CHECK(!ExistsFile(JoinPaths(options_.workspace_path, "database.db")));

    // Sparse reconstruction directories. Assume direcotry name "sparse" is used.
    const std::string input_path1 = JoinPaths(options_.workspace_path1, "sparse", "merged-ba");
    const std::string input_path2 = JoinPaths(options_.workspace_path2, "sparse", "merged-ba");
    output_path_ = JoinPaths(options_.workspace_path, "sparse", "merged_ba");
    CHECK(ExistsDir(input_path1));
    CHECK(ExistsDir(input_path2));
    CreateDirIfNotExists(output_path_);

    reconstruction1_.Read(input_path1);
    reconstruction2_.Read(input_path2);
}

void IndependentModelMergerController::Stop() {
    if(active_thread_ != nullptr) {
        active_thread_->Stop();
    }
    Thread::Stop();
}

void IndependentModelMergerController::Run() {
    if(IsStopped()) {
        return;
    }

    RunCopyImages();

    if(IsStopped()) {
        return;
    }

    RunMergeDatabase();

    if(IsStopped()) {
        return;
    }

    RunFeatureMatching();

    if(IsStopped()) {
        return;
    }

    RunModelMerger();

    if(IsStopped()) {
        return;
    }

    if(options_.global_ba) {
        RunGlobalBundleAdjuster();
    }
}

void IndependentModelMergerController::RunCopyImages() {
    PrintHeading1("Copying images used in two models together.");
    Timer timer;
    timer.Start();

    const std::string model_name1 = GetPathBaseName(options_.workspace_path1);
    const std::string model_name2 = GetPathBaseName(options_.workspace_path2);
    const std::string image_path1 = JoinPaths(options_.workspace_path1, "images");
    const std::string image_path2 = JoinPaths(options_.workspace_path2, "images");
    const std::string dst_image_path = JoinPaths(options_.workspace_path, "images");

    RemoveDirContent(dst_image_path);

    std::cout << StringPrintf("  Copying images from model %s...",
                              model_name1.c_str())
              << std::endl;
    size_t num_images_model1 = CopyDir(image_path1, dst_image_path);
    std::cout << StringPrintf("    %d images are copied",
                              num_images_model1)
              << std::endl;

    std::cout << StringPrintf("  Copying images from model %s...",
                              model_name2.c_str())
              << std::endl;
    size_t num_images_model2 = CopyDir(image_path2, dst_image_path);
    std::cout << StringPrintf("    %d images are copied",
                              num_images_model2)
              << std::endl;

    option_manager_.cross_group_matching->first_group_size = num_images_model1;

    timer.PrintSeconds();
}

void IndependentModelMergerController::RunMergeDatabase() {
    PrintHeading1("Merging database used in two models together");
    Timer timer;
    timer.Start();

    const std::string database_path = JoinPaths(options_.workspace_path, "database.db");
    const std::string database_path1 = JoinPaths(options_.workspace_path1, "database.db");
    const std::string database_path2 = JoinPaths(options_.workspace_path2, "database.db");

    Database database1 (database_path1);
    std::unordered_map<camera_t, camera_t> camera_id_map1;
    std::unordered_map<image_t, image_t> image_id_map1;
    std::unordered_map<image_pair_t, image_pair_t> image_pair_id_map1;
    PrintHeading2("Database 1: " + database_path1);
    std::cout << StringPrintf("Images: %d", database1.NumImages())
              << std::endl;

    Database database2 (database_path2);
    std::unordered_map<camera_t, camera_t> camera_id_map2;
    std::unordered_map<image_t, image_t> image_id_map2;
    std::unordered_map<image_pair_t, image_pair_t> image_pair_id_map2;
    PrintHeading2("Database 2: " + database_path2);
    std::cout << StringPrintf("Images: %d", database2.NumImages())
              << std::endl;

    Database database (database_path);

    database.Combine(database1, &camera_id_map1, &image_id_map1, &image_pair_id_map1, 
                     options_.reuse_camera);
    database.Combine(database2, &camera_id_map2, &image_id_map2, &image_pair_id_map2, 
                     options_.reuse_camera);

    database1.Close();
    database2.Close();
    database.Close();

    *option_manager_.database_path = database_path;

    timer.PrintSeconds();

    PrintHeading1("Updating indices in two sparse models");
    timer.Restart();

    PrintHeading2("Reconstruction 1");
    std::cout << StringPrintf("Images: %d", reconstruction1_.NumRegImages())
              << std::endl;
    std::cout << StringPrintf("Points: %d", reconstruction1_.NumPoints3D())
              << std::endl;

    PrintHeading2("Reconstruction 2");
    std::cout << StringPrintf("Images: %d", reconstruction2_.NumRegImages())
              << std::endl;
    std::cout << StringPrintf("Points: %d", reconstruction2_.NumPoints3D())
              << std::endl;

    reconstruction1_.UpdateIndices(camera_id_map1, image_id_map1,
                                   image_pair_id_map1);
    reconstruction2_.UpdateIndices(camera_id_map2, image_id_map2,
                                   image_pair_id_map2);

    std::cout << std::endl;
    timer.PrintSeconds();
}

void IndependentModelMergerController::RunFeatureMatching() {
    CHECK(option_manager_.database_path);
    CHECK(ExistsFile(*option_manager_.database_path));
    cross_group_matcher_.reset(new CrossGroupFeatureMatcher(
                                   *option_manager_.cross_group_matching,
                                   *option_manager_.vocab_tree_matching,
                                   *option_manager_.sift_matching,
                                   *option_manager_.database_path));

    active_thread_ = cross_group_matcher_.get();
    cross_group_matcher_->Start();
    cross_group_matcher_->Wait();
    cross_group_matcher_.reset();
    active_thread_ = nullptr;
}

void IndependentModelMergerController::RunModelMerger() {
    PrintHeading1("Registering images");
    Timer timer;
    timer.Start();

    // Assume first model has the more registered images.
    if(reconstruction1_.NumRegImages() < reconstruction2_.NumRegImages()) {
        std::swap(reconstruction1_, reconstruction2_);
    }

    DatabaseCache database_cache;
    {
        CHECK(option_manager_.database_path);
        CHECK(ExistsFile(*option_manager_.database_path));
        Database database(*option_manager_.database_path);
        Timer timer;
        timer.Start();
        const size_t min_num_matches =
            static_cast<size_t>(option_manager_.mapper->min_num_matches);
        database_cache.Load(database, min_num_matches,
                            option_manager_.mapper->ignore_watermarks,
                            option_manager_.mapper->image_names);
    }

    // Register each frame in reconstruction2 to reconstruction1.
    IncrementalMapper mapper(&database_cache);
    mapper.BeginReconstruction(&reconstruction1_);

    mapper.CopyCamerasFrom(reconstruction1_);
    mapper.CopyCamerasFrom(reconstruction2_);

    const auto mapper_options = option_manager_.mapper->Mapper();
    for(const image_t& image_id : reconstruction2_.RegImageIds()) {
        Image image_in_rec1 = reconstruction1_.Image(image_id);
        if(image_in_rec1.IsRegistered()) {
            PrintHeading2("Image #" + std::to_string(image_id) +
                          " is registered.");
            continue;
        } else {
            PrintHeading2("Registering image #" + std::to_string(image_id) + " (" +
                          std::to_string(reconstruction1_.NumRegImages() + 1) + ")");

            std::cout << "  => Image sees " << image_in_rec1.NumVisiblePoints3D()
                      << " / " << image_in_rec1.NumObservations() << " points"
                      << std::endl;

            if(mapper.RegisterNextImage(mapper_options, image_id)) {
                std::cout << "succeeded." << std::endl;
            } else {
                std::cout << "failed." << std::endl;
            }
        }
    }

    const bool kDiscardReconstruction = false;
    mapper.EndReconstruction(kDiscardReconstruction);

    timer.PrintSeconds();

    // Merging reconstruction, different from model_merger for re-transform
    // outliner registered images and missing images instead of missing
    // images only.
    PrintHeading1("Merging models");
    timer.Restart();
    
    if(reconstruction1_.Merge(reconstruction2_, options_.max_reproj_error, true)) {
        std::cout << "=> Merge succeeded" << std::endl;
        PrintHeading2("Merged reconstruction");
        std::cout << StringPrintf("Images: %d", reconstruction1_.NumRegImages())
                  << std::endl;
        std::cout << StringPrintf("Points: %d", reconstruction1_.NumPoints3D())
                  << std::endl;
        // Only write reconstruction if succeeded.
        CreateDirIfNotExists(output_path_);
        reconstruction1_.Write(output_path_);
    } else {
        std::cout << "=> Merge failed" << std::endl;
    }

    std::cout << std::endl;
    timer.PrintSeconds();
}

void IndependentModelMergerController::RunGlobalBundleAdjuster() {
    Timer timer;
    timer.Start();

    BundleAdjustmentController ba_controller(option_manager_, &reconstruction1_);
    active_thread_ = &ba_controller;
    ba_controller.Start();
    ba_controller.Wait();
    active_thread_ = nullptr;

    CreateDirIfNotExists(output_path_);
    reconstruction1_.Write(output_path_);
}

}  // namespace colmap
