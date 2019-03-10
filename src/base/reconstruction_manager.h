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

#ifndef COLMAP_SRC_BASE_RECONSTRUCTION_MANAGER_H_
#define COLMAP_SRC_BASE_RECONSTRUCTION_MANAGER_H_

#include "base/reconstruction.h"

namespace colmap
{

class OptionManager;

class ReconstructionManager
{
public:
    ReconstructionManager();

    // Move constructor and assignment.
    ReconstructionManager ( ReconstructionManager&& other );
    ReconstructionManager& operator= ( ReconstructionManager&& other );

    // The number of reconstructions managed.
    size_t Size() const;

    // Get a reference to a specific reconstruction.
    const Reconstruction& Get ( const size_t idx ) const;
    Reconstruction& Get ( const size_t idx );

    // Add a new empty reconstruction and return its index.
    size_t Add();

    // Add an existed reconstruction to manager. This function intentionally
    // copy the input reconstruction.
    size_t Add ( Reconstruction reconstruction );

    // Delete a specific reconstruction.
    void Delete ( const size_t idx );

    // Delete all reconstructions.
    void Clear();

    // Read and add a new reconstruction and return its index.
    size_t Read ( const std::string& path );
    
    // Read all previously managed reconstructions from files. Sub-folders
    // "0", "1", "2", ... are assumed. It will probe each folder starting
    // from "0", until a folder doesn't exist.
    void ReadAll ( const std::string& path );

    // Write all managed reconstructions into sub-folders "0", "1", "2", ...
    // If the option manager object is not null, the options are written
    // to each respective reconstruction folder as well.
    void Write ( const std::string& path, const OptionManager* options ) const;

private:
    NON_COPYABLE ( ReconstructionManager )

    std::vector<std::unique_ptr<Reconstruction>> reconstructions_;
};

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_RECONSTRUCTION_MANAGER_H_
