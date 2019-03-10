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

#ifndef COLMAP_SRC_CONTROLLERS_INDEPENDENT_MODEL_MERGER_H_
#define COLMAP_SRC_CONTROLLERS_INDEPENDENT_MODEL_MERGER_H_

#include <string>

#include "base/reconstruction.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace colmap {

class IndependentModelMergerController : public Thread
{
public:
    struct Options {
        // The path to the workspace folder in which all results are stored.
        std::string workspace_path;

        // The path to the workspace folder storing all results from model1.
        std::string workspace_path1;

        // The path to the workspace folder storing all results from model2.
        std::string workspace_path2;
        
        // The path to vocabulary tree.
        std::string vocab_tree_path = "";

        // The max reprojection error for model merging. See function
        // ComputeAlignmentBetweenReconstructions in SimilarityTransform.h
        double max_reproj_error = 64.0;

        // Whether run a global bundle adjuster at the end.
        bool global_ba = true;

        // Whether to use the GPU in feature matching.
        bool use_gpu = true;

        // Index of the GPU used for GPU stages. For multi-GPU computation,
        // you should separate multiple GPU indices by comma, e.g., "0,1,2,3".
        // By default, all GPUs will be used in all stages.
        std::string gpu_index = "-1";
    };

    IndependentModelMergerController(const Options& options,
                                     const OptionManager& option_manager);

    void Stop() override;

private:
    void Run() override;
    void RunCopyImages();
    void RunMergeDatabase();
    void RunFeatureMatching();
    void RunModelMerger();
    void RunGlobalBundleAdjuster();

    Reconstruction reconstruction1_;
    Reconstruction reconstruction2_;
    Reconstruction main_reconstruction_;

    const Options options_;
    OptionManager option_manager_;
    Thread* active_thread_;
    std::unique_ptr<Thread> cross_group_matcher_;
};

} // namespace colmap

#endif  // COLMAP_SRC_CONTROLLERS_INDEPENDENT_MODEL_MERGER_H_
