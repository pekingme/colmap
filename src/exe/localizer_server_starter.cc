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
#include <string>
#include <signal.h>
#include <unistd.h>

#include <QApplication>

#include "util/option_manager.h"
#include "util/misc.h"
#include "base/reconstruction.h"
#include "base/image_reader.h"
#include "base/camera_models.h"
#include "base/database_cache.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "controllers/incremental_mapper.h"
#include "server/rest_server.h"

using namespace colmap;

int main (int argc, char** argv) {
    // Initialize QApplication is OpenGL needed.
    std::unique_ptr<QApplication> app;
    if(kUseOpenGLx) {
        app.reset(new QApplication(argc, argv));
    }
    
    // Create and start server.
    int port_num = 34567;
    int thread_num = 2;
    RestServer server (port_num, thread_num);
    
    server.Start();
    
    std::string line;
    getline(std::cin, line);
    
    server.Shutdown();
    
    return EXIT_SUCCESS;
}
