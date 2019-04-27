
set(FOLDER_NAME "server")

COLMAP_ADD_SOURCES(
    localizer.h localizer.cc
    localization_result.h localization_result.cc
    landmark_info.h landmark_info.cc
    azure_blob_loader.h azure_blob_loader.cc
    rest_handler.h rest_handler.cc
    rest_server.h rest_server.cc
)
