///-------------------------------------------------------------------------------------------------
// file:	ResultSettings.cpp
//
// summary:	Implements the result settings class
///-------------------------------------------------------------------------------------------------
#include "ResultSettings.h"

#include <SerAr/SerAr.hpp>

#include <fmt/format.h>

namespace Settings
{
        const std::map<IResultFileType, std::string> IResultFileTypeMap = { { { IResultFileType::ResultFileType_undefined,"undefined" },
                                                                              { IResultFileType::ResultFileType_MATLAB,"MATLAB" },
                                                                              { IResultFileType::ResultFileType_HDF5 ,"HDF5" },
                                                                              { IResultFileType::ResultFileType_JSON ,"JSON" } } };

        const std::map<IResultFileType, SerAr::ArchiveTypeEnum> ResultFileEnumToArchiveEnumMap = 
            { { { IResultFileType::ResultFileType_MATLAB, SerAr::ArchiveTypeEnum::MATLAB },
                { IResultFileType::ResultFileType_HDF5, SerAr::ArchiveTypeEnum::HDF5 },
                { IResultFileType::ResultFileType_JSON, SerAr::ArchiveTypeEnum::JSON } } };

        std::string to_string(const IResultFileType& field)
        {
            return IResultFileTypeMap.at(field);
        }

        template<>
        IResultFileType from_string<IResultFileType>(const std::string &str)
        {
            for (auto it : IResultFileTypeMap)
                if (it.second == str)
                    return it.first;

            throw std::runtime_error{ fmt::format("ResultSettings: Type of result file unknown! (Value:'{}')", str) };
        }

    std::string ResultSettings::getExtensionFromType() const
    {
        if(m_ResultFileType == IResultFileType::ResultFileType_undefined)
            throw std::runtime_error{ "ResultSettings: ResultFileType not defined! " };

        return std::string{SerAr::getArchiveDefaultExtension(ResultFileEnumToArchiveEnumMap.at(m_ResultFileType))};
    }
}
