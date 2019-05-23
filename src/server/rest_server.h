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

#ifndef RESTSERVER_H
#define RESTSERVER_H

#include <pistache/http.h>
#include <pistache/endpoint.h>
#include <QApplication>

#include "server/rest_handler.h"

using namespace Pistache;

class RestServer
{
public:
    RestServer(int port_num, int thread_num);
    
    void Start();
    
    void Shutdown();
private:
    
    std::shared_ptr<Http::Endpoint> server_;
    std::unique_ptr<QApplication> app_;
};

#endif // RESTSERVER_H
