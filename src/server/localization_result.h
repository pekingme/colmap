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

#ifndef LOCALIZATION_RESULT_H_
#define LOCALIZATION_RESULT_H_

#include <iostream>

#include <Eigen/Core>

#include "util/types.h"
#include "base/image.h"
#include "rapidjson/document.h"
#include "server/constants.h"

using namespace std;
using namespace colmap;

/**
 * This class is the holder of localization result using one image.
 */
class LocalizationResult
{
public:
    LocalizationResult ( const Image& image, const bool success )
        : image_id_ ( image.ImageId() ), image_name_ ( image.Name() ), success_ ( success )
    {
        if ( success_ ) {
            projection_center_ = Eigen::Vector3d ( image.ProjectionCenter() );
            viewing_direction_ = Eigen::Vector3d ( image.ViewingDirection() );
        } else {
            projection_center_ = Eigen::Vector3d::Zero();
            viewing_direction_ = Eigen::Vector3d::Zero();
        }
    }

    inline LocalizationResult Scale ( const double scale ) const
    {
        return LocalizationResult ( image_id_, image_name_, success_,
                                    projection_center_ * scale,
                                    viewing_direction_ * scale );
    }

    rapidjson::Value AsJSON(rapidjson::Document* document) const;

private:
    LocalizationResult ( const image_t image_id, const string image_name, bool success,
                         const Eigen::Vector3d& projection_center,
                         const Eigen::Vector3d& viewing_direction )
        : image_id_ ( image_id ), image_name_ ( image_name ), success_ ( success ),
          projection_center_ ( projection_center ), viewing_direction_ ( viewing_direction ) {}

    image_t image_id_;
    string image_name_;
    bool success_ = false;

    Eigen::Vector3d projection_center_;
    Eigen::Vector3d viewing_direction_;

    friend std::ostream& operator<< ( std::ostream& os, const LocalizationResult& data )
    {
        os << "Image_" << data.image_id_ << " [" << data.image_name_ << "]: ";
        if ( data.success_ ) {
            os << "successful" << endl << data.projection_center_ << endl
               << data.viewing_direction_ << endl;
        } else {
            os << "failed" << endl;
        }
        return os;
    }
};

#endif // LOCALIZATION_RESULT_H_
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
