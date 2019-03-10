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

#ifndef LANDMARK_INFO_H_
#define LANDMARK_INFO_H_

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "server/cpprest_import.h"

/**
 * An entity class holding landmark inforamtion.
 */
class LandmarkInfo
{
public:
    LandmarkInfo(const std::string& name, const double x, const double y, const double z)
        : name_(name), x_(x), y_(y), z_(z) {}
        
    inline LandmarkInfo Scale(const double scale){
        return LandmarkInfo(name_, x_ * scale, y_ * scale, z_ * scale);
    }

    web::json::value AsJSON() const;

private:
    std::string name_;
    double x_;
    double y_;
    double z_;
};

#endif // LANDMARK_INFO_H_
