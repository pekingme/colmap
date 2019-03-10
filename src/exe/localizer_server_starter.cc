// Copyright (c) 2019, Hao Dong.
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
// Author: Hao Dong (haod at ecs umass edu)

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <iostream>

#include <QApplication>

#include "server/cpprest_import.h"

#include "util/option_manager.h"
#include "util/misc.h"
#include "base/reconstruction.h"
#include "base/image_reader.h"
#include "base/camera_models.h"
#include "base/database_cache.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "controllers/incremental_mapper.h"
#include "server/localizer_server.h"

using namespace colmap;
using namespace utility;
using namespace web;

int main (int argc, char** argv) {
    
    // Initialize localizer with socket.
    utility::string_t port = "34567";
    if(argc == 2){
        port = argv[1];
    }
    
    utility::string_t address = "http://128.119.86.65:";
    address.append(port);
    
    uri_builder uri(address);
    uri.append_path("api");
    
    auto addr = uri.to_uri().to_string();
    
    // Initialize QApplication is OpenGL needed.
    std::unique_ptr<QApplication> app;
    if(kUseOpenGL) {
        app.reset(new QApplication(argc, argv));
    }
    
    std::unique_ptr<LocalizerServer> localizer_server (new LocalizerServer(addr));
    localizer_server->open().wait();
    
    std::cout << utility::string_t("Listening for requests at: ") << addr << std::endl;
    
    std::cout << "Press ENTER to exit." << std::endl;
    
    std::string line;
    std::getline(std::cin, line);
    
    localizer_server->close().wait();
    
    return EXIT_SUCCESS;
}
