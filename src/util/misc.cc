// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
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
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "util/misc.h"

#include <cstdarg>
#include <fstream>

#include <boost/algorithm/string.hpp>

namespace colmap
{
    
void LogTestingData(const std::string& data){
    std::ofstream outfile;
    outfile.open("~/Desktop/testing_data.txt",std::ios_base::app);
    outfile << data << std::endl;
    outfile.close();
}
    
std::string EnsureTrailingSlash ( const std::string& str )
{
    if ( str.length() > 0 ) {
        if ( str.back() != '/' ) {
            return str + "/";
        }
    } else {
        return str + "/";
    }
    return str;
}

bool HasFileExtension ( const std::string& file_name, const std::string& ext )
{
    CHECK ( !ext.empty() );
    CHECK_EQ ( ext.at ( 0 ), '.' );
    std::string ext_lower = ext;
    StringToLower ( &ext_lower );
    if ( file_name.size() >= ext_lower.size() &&
            file_name.substr ( file_name.size() - ext_lower.size(), ext_lower.size() ) ==
            ext_lower ) {
        return true;
    }
    return false;
}

void SplitFileExtension ( const std::string& path, std::string* root,
                          std::string* ext )
{
    const auto parts = StringSplit ( path, "." );
    CHECK_GT ( parts.size(), 0 );
    if ( parts.size() == 1 ) {
        *root = parts[0];
        *ext = "";
    } else {
        *root = "";
        for ( size_t i = 0; i < parts.size() - 1; ++i ) {
            *root += parts[i] + ".";
        }
        *root = root->substr ( 0, root->length() - 1 );
        if ( parts.back() == "" ) {
            *ext = "";
        } else {
            *ext = "." + parts.back();
        }
    }
}

bool ExistsFile ( const std::string& path )
{
    return boost::filesystem::is_regular_file ( path );
}

bool ExistsDir ( const std::string& path )
{
    return boost::filesystem::is_directory ( path );
}

bool ExistsPath ( const std::string& path )
{
    return boost::filesystem::exists ( path );
}

void CreateDirIfNotExists ( const std::string& path )
{
    if ( !ExistsDir ( path ) ) {
        const std::string base_path = GetParentDir ( path );
        CreateDirIfNotExists ( base_path );
        boost::filesystem::create_directory ( path );
        CHECK ( ExistsDir ( path ) );
    }
}

std::string GenerateRandomString ( const int length )
{
    static const char alphanum[] = "1234567890"
                                   "QWERTYUIOPASDFGHJKLZXCVBNM"
                                   "qwertyuiopasdfghjklzxcvbnm";

    std::string result = "";
    for ( int i=0; i<length; i++ ) {
        result.push_back ( alphanum[rand() % ( sizeof ( alphanum )-1 )] );
    }
    return result;
}

std::string GetWorkingDirectory()
{
    return boost::filesystem::current_path().string();
}

std::string GetPathBaseName ( const std::string& path )
{
    const std::vector<std::string> names =
        StringSplit ( StringReplace ( path, "\\", "/" ), "/" );
    if ( names.size() > 1 && names.back() == "" ) {
        return names[names.size() - 2];
    } else {
        return names.back();
    }
}

std::string GetParentDir ( const std::string& path )
{
    return boost::filesystem::path ( path ).parent_path().string();
}

std::string GetRelativePath ( const std::string& from, const std::string& to )
{
    // This implementation is adapted from:
    // https://stackoverflow.com/questions/10167382
    // A native implementation in boost::filesystem is only available starting
    // from boost version 1.60.
    using namespace boost::filesystem;

    path from_path = canonical ( path ( from ) );
    path to_path = canonical ( path ( to ) );

    // Start at the root path and while they are the same then do nothing then
    // when they first diverge take the entire from path, swap it with '..'
    // segments, and then append the remainder of the to path.
    path::const_iterator from_iter = from_path.begin();
    path::const_iterator to_iter = to_path.begin();

    // Loop through both while they are the same to find nearest common directory
    while ( from_iter != from_path.end() && to_iter != to_path.end() &&
            ( *to_iter ) == ( *from_iter ) ) {
        ++ to_iter;
        ++ from_iter;
    }

    // Replace from path segments with '..' (from => nearest common directory)
    path rel_path;
    while ( from_iter != from_path.end() ) {
        rel_path /= "..";
        ++ from_iter;
    }

    // Append the remainder of the to path (nearest common directory => to)
    while ( to_iter != to_path.end() ) {
        rel_path /= *to_iter;
        ++ to_iter;
    }

    return rel_path.string();
}

std::vector<std::string> GetFileList ( const std::string& path )
{
    std::vector<std::string> file_list;
    for ( auto it = boost::filesystem::directory_iterator ( path );
            it != boost::filesystem::directory_iterator(); ++it ) {
        if ( boost::filesystem::is_regular_file ( *it ) ) {
            const boost::filesystem::path file_path = *it;
            file_list.push_back ( file_path.string() );
        }
    }
    return file_list;
}

std::vector<std::string> GetRecursiveFileList ( const std::string& path )
{
    std::vector<std::string> file_list;
    for ( auto it = boost::filesystem::recursive_directory_iterator ( path );
            it != boost::filesystem::recursive_directory_iterator(); ++it ) {
        if ( boost::filesystem::is_regular_file ( *it ) ) {
            const boost::filesystem::path file_path = *it;
            file_list.push_back ( file_path.string() );
        }
    }
    return file_list;
}

std::vector<std::string> GetDirList ( const std::string& path )
{
    std::vector<std::string> dir_list;
    for ( auto it = boost::filesystem::directory_iterator ( path );
            it != boost::filesystem::directory_iterator(); ++it ) {
        if ( boost::filesystem::is_directory ( *it ) ) {
            const boost::filesystem::path dir_path = *it;
            dir_list.push_back ( dir_path.string() );
        }
    }
    return dir_list;
}

std::vector<std::string> GetDirListNameOnly(const std::string& path){
    std::vector<std::string> dir_list = GetDirList(path);
    std::vector<std::string> dir_name_list;
    for(auto dir : dir_list){
        boost::filesystem::path path(dir);
        dir_name_list.emplace_back(path.filename().string());
    }
    return dir_name_list;
}

std::vector<std::string> GetRecursiveDirList ( const std::string& path )
{
    std::vector<std::string> dir_list;
    for ( auto it = boost::filesystem::recursive_directory_iterator ( path );
            it != boost::filesystem::recursive_directory_iterator(); ++it ) {
        if ( boost::filesystem::is_directory ( *it ) ) {
            const boost::filesystem::path dir_path = *it;
            dir_list.push_back ( dir_path.string() );
        }
    }
    return dir_list;
}

size_t GetFileSize ( const std::string& path )
{
    std::ifstream file ( path, std::ifstream::ate | std::ifstream::binary );
    CHECK ( file.is_open() ) << path;
    return file.tellg();
}

void RemoveDirContent ( const std::string& path, const bool remove_itself )
{
    if ( ExistsDir ( path ) ) {
        if ( remove_itself ) {
            boost::filesystem::remove_all ( path );
        } else {
            for ( auto it = boost::filesystem::directory_iterator ( path );
                    it != boost::filesystem::directory_iterator(); ++it ) {
                boost::filesystem::remove_all ( *it );
            }
        }
    }
}

size_t CopyDir ( const std::string& from_path, const std::string& to_path )
{
    CHECK ( ExistsDir ( from_path ) );
    CHECK ( ExistsDir ( to_path ) );

    return CopyDirRecursive ( boost::filesystem::path ( from_path ),
                              boost::filesystem::path ( to_path ) );
}

size_t CopyDirRecursive ( const boost::filesystem::path& from_path,
                          const boost::filesystem::path& to_path )
{
    size_t total_copied = 0;

    for ( auto it = boost::filesystem::directory_iterator ( from_path );
            it != boost::filesystem::directory_iterator(); ++it ) {
        boost::filesystem::path to_file_path = to_path / it->path().filename();
        if ( boost::filesystem::is_directory ( *it ) ) {
            boost::filesystem::create_directory ( to_file_path );
            total_copied += CopyDirRecursive ( it->path(), to_file_path );
        } else if ( boost::filesystem::is_regular_file ( *it ) ) {
            boost::filesystem::copy ( it->path(), to_file_path );
            total_copied ++;
        }
    }

    return total_copied;
}

void PrintHeading1 ( const std::string& heading )
{
    std::cout << std::endl << std::string ( 78, '=' ) << std::endl;
    std::cout << heading << std::endl;
    std::cout << std::string ( 78, '=' ) << std::endl << std::endl;
}

void PrintHeading2 ( const std::string& heading )
{
    std::cout << std::endl << heading << std::endl;
    std::cout << std::string ( std::min<int> ( heading.size(), 78 ), '-' ) << std::endl;
}

template <>
std::vector<std::string> CSVToVector ( const std::string& csv )
{
    auto elems = StringSplit ( csv, ",;" );
    std::vector<std::string> values;
    values.reserve ( elems.size() );
    for ( auto& elem : elems ) {
        StringTrim ( &elem );
        if ( elem.empty() ) {
            continue;
        }
        values.push_back ( elem );
    }
    return values;
}

template <>
std::vector<int> CSVToVector ( const std::string& csv )
{
    auto elems = StringSplit ( csv, ",;" );
    std::vector<int> values;
    values.reserve ( elems.size() );
    for ( auto& elem : elems ) {
        StringTrim ( &elem );
        if ( elem.empty() ) {
            continue;
        }
        try {
            values.push_back ( std::stoi ( elem ) );
        } catch ( std::exception ) {
            return std::vector<int> ( 0 );
        }
    }
    return values;
}

template <>
std::vector<float> CSVToVector ( const std::string& csv )
{
    auto elems = StringSplit ( csv, ",;" );
    std::vector<float> values;
    values.reserve ( elems.size() );
    for ( auto& elem : elems ) {
        StringTrim ( &elem );
        if ( elem.empty() ) {
            continue;
        }
        try {
            values.push_back ( std::stod ( elem ) );
        } catch ( std::exception ) {
            return std::vector<float> ( 0 );
        }
    }
    return values;
}

template <>
std::vector<double> CSVToVector ( const std::string& csv )
{
    auto elems = StringSplit ( csv, ",;" );
    std::vector<double> values;
    values.reserve ( elems.size() );
    for ( auto& elem : elems ) {
        StringTrim ( &elem );
        if ( elem.empty() ) {
            continue;
        }
        try {
            values.push_back ( std::stold ( elem ) );
        } catch ( std::exception ) {
            return std::vector<double> ( 0 );
        }
    }
    return values;
}

std::vector<std::string> ReadTextFileLines ( const std::string& path )
{
    std::ifstream file ( path );
    CHECK ( file.is_open() ) << path;

    std::string line;
    std::vector<std::string> lines;
    while ( std::getline ( file, line ) ) {
        StringTrim ( &line );

        if ( line.empty() ) {
            continue;
        }

        lines.push_back ( line );
    }

    return lines;
}

void RemoveCommandLineArgument ( const std::string& arg, int* argc, char** argv )
{
    for ( int i = 0; i < *argc; ++i ) {
        if ( argv[i] == arg ) {
            for ( int j = i + 1; j < *argc; ++j ) {
                argv[i] = argv[j];
            }
            *argc -= 1;
            break;
        }
    }
}

void PrintMemoryUsage()
{
    double vm_usage     = 0.0;
    double resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs ( "/proc/self/stat", std::ios_base::in );
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> vsize >> rss;
    }

    long page_size_kb = sysconf ( _SC_PAGE_SIZE ) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;

    PrintHeading2 ( "Process memory usage" );
    std::cout << "  virtual memory usage: " << vm_usage << std::endl;
    std::cout << "  resident set size: " << resident_set << std::endl;
}

void ProcessMemoryUsage ( double* vm_usage, double* resident_set )
{
    *vm_usage     = 0.0;
    *resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs ( "/proc/self/stat", std::ios_base::in );
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> vsize >> rss;
    }

    long page_size_kb = sysconf ( _SC_PAGE_SIZE ) / 1024; // in case x86-64 is configured to use 2MB pages
    *vm_usage = vsize / 1024.0;
    *resident_set = rss * page_size_kb;
}

}  // namespace colmap
