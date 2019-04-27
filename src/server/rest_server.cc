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

#include "rest_server.h"
#include "util/misc.h"

using namespace colmap;

RestServer::RestServer ( int port_num, int thread_num )
{
    // Create server.
    Port port ( port_num );
    Address addr ( Ipv4::any(), port );

    server_ = std::make_shared<Http::Endpoint> ( addr );

    auto opts = Http::Endpoint::options().threads ( thread_num )
                .flags ( Tcp::Options::InstallSignalHandler );

    server_->init ( opts );
    server_->setHandler ( Http::make_handler<RestHandler>() );
    PrintHeading1 ( "Restful server API" );
    std::cout << addr.host() << ":" << static_cast<uint16_t> ( addr.port() ) << std::endl;
}

void RestServer::Start()
{
    std::cout << "  started." << std::endl;
    server_->serveThreaded();
}

void RestServer::Shutdown()
{
    server_->shutdown();
    std::cout << "Restful server terminated" << std::endl;
}
