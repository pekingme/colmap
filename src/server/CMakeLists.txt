
set(FOLDER_NAME "server")

COLMAP_ADD_SOURCES(
    cpprest_import.h
    localizer_server.h localizer_server.cc
    localizer.h localizer.cc
    localization_result.h localization_result.cc
    landmark_info.h landmark_info.cpp
)
