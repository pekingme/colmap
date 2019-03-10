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

#include "server/localization_result.h"

inline web::json::value Vector3DAsJSON(const Eigen::Vector3d vector){
    web::json::value result = web::json::value::object();
    result["x"] = web::json::value::number(vector.x());
    result["y"] = web::json::value::number(vector.y());
    result["z"] = web::json::value::number(vector.z());
    
    return result;
}

web::json::value LocalizationResult::AsJSON() const
{
    web::json::value result = web::json::value::object();
    result[kIMAGE_ID] = web::json::value::number(image_id_);
    result[kIMAGE_NAME] = web::json::value::string(image_name_);
    result[kSUCCESS] = web::json::value::boolean(success_);
    result[kPROJECT_CENTER] = Vector3DAsJSON(projection_center_);
    result[kVIEW_DIRECTION] = Vector3DAsJSON(viewing_direction_);
    return result;
}
