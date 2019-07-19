#pragma once

#include <string>

#include "AzureStorageCpplite/storage_EXPORTS.h"

#include "AzureStorageCpplite/http_base.h"
#include "AzureStorageCpplite/storage_account.h"
#include "AzureStorageCpplite/storage_request_base.h"

namespace azure {  namespace storage_lite {

    class delete_blob_request_base : public blob_request_base
    {
    public:
        enum class delete_snapshots
        {
            unspecified,
            include,
            only
        };

        virtual std::string container() const = 0;
        virtual std::string blob() const = 0;

        virtual std::string snapshot() const { return std::string(); }
        virtual delete_snapshots ms_delete_snapshots() const { return delete_snapshots::unspecified; }

        AZURE_STORAGE_API void build_request(const storage_account &a, http_base &h) const override;
    };

}}  // azure::storage_lite
