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

#include "landmark_info.h"

rapidjson::Value LandmarkInfo::AsJSON(rapidjson::Document* document) const
{
    using namespace rapidjson;
    using rapidjson::Type;

    Value value ( kObjectType );
    value.AddMember ( Value ( "name", document->GetAllocator() ).Move(),
                      Value ( StringRef(name_.c_str()) ).Move(),
                      document->GetAllocator() );
    value.AddMember(Value("x", document->GetAllocator()).Move(),
                    Value(x_).Move(),
                    document->GetAllocator());
    value.AddMember(Value("y", document->GetAllocator()).Move(),
                    Value(y_).Move(),
                    document->GetAllocator());
    value.AddMember(Value("z", document->GetAllocator()).Move(),
                    Value(z_).Move(),
                    document->GetAllocator());
    
    return value;
}
