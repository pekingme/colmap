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

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

// OpenGL indicator.
#ifdef CUDA_ENABLED
const bool kUseOpenGLx = false;
#else
const bool kUseOpenGLx = true;
#endif

// For localizer use.
const std::string kLocalizationReconstructionFolder = "aligned";
const std::string kLocalizationDatabaseFilename = "database.db";
const std::string kLocalizationImagesFolder = "images";
const std::string kLocalizationIndexingFilename = "index.bin";
const std::string kLocalizationIndexingImageListFilename = "database_image_list.txt";

// For serving graph use.
const std::string kGraphServingGraphFilename = "graph.json";

// For camera calibration use.
const std::string kCalibrationWorkspace = "calibration/";
const std::string kCalibrationImageFolder = "pictures/";
const std::string kCalibrationReconstructionFolder = "sparse/";
const std::string kCalibrationDatabaseFilename = "database.db";
const std::string kCalibrationDefaultCameraModel = "RADIAL_FISHEYE";

// For parsing localization result to JSON
static const std::string kLocResultImageId = "image_id";
static const std::string kLocResultImageName = "image_name";
static const std::string kLocResultSuccess = "success";
static const std::string kLocResultProjectCenter = "project_center";
static const std::string kLocResultViewDirection = "view_direction";

// For parsing camera calibration result to JSON
static const std::string kCalibResultCameraModel = "camera_model";
static const std::string kCalibResultCameraParams = "camera_params";

#endif // CONSTANTS_H
