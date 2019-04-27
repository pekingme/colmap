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

inline rapidjson::Value Vector3DAsJSON( const Eigen::Vector3d vector,
        rapidjson::Document* document )
{
    using namespace rapidjson;
    using rapidjson::Type;

    Value value ( kObjectType );
    value.AddMember ( Value ( "x", document->GetAllocator() ).Move(),
                      Value ( vector.x() ),
                      document->GetAllocator() );
    value.AddMember ( Value ( "y", document->GetAllocator() ).Move(),
                      Value ( vector.y() ),
                      document->GetAllocator() );
    value.AddMember ( Value ( "z", document->GetAllocator() ).Move(),
                      Value ( vector.z() ),
                      document->GetAllocator() );

    return value;
}

rapidjson::Value LocalizationResult::AsJSON ( rapidjson::Document* document ) const
{
    using namespace rapidjson;
    using rapidjson::Type;

    Value value ( kObjectType );
    value.AddMember ( Value ( kImageId.c_str(), document->GetAllocator() ).Move(),
                      Value ( image_id_ ).Move(),
                      document->GetAllocator() );
    value.AddMember ( Value ( kImageName.c_str(), document->GetAllocator() ).Move(),
                      Value ( image_name_.c_str(), document->GetAllocator() ).Move(),
                      document->GetAllocator() );
    value.AddMember ( Value ( kSuccess.c_str(), document->GetAllocator() ).Move(),
                      Value ( success_ ),
                      document->GetAllocator() );
    value.AddMember ( Value ( kProjectCenter.c_str(), document->GetAllocator() ).Move(),
                      Vector3DAsJSON ( projection_center_, document ),
                      document->GetAllocator() );
    value.AddMember ( Value ( kViewDirection.c_str(), document->GetAllocator() ).Move(),
                      Vector3DAsJSON ( viewing_direction_, document ),
                      document->GetAllocator() );

    return value;
}
