///-------------------------------------------------------------------------------------------------
// file:	ResultSettings.cpp
//
// summary:	Implements the result settings class
///-------------------------------------------------------------------------------------------------
#include "ResultSettings.h"

#include <SerAr/AllArchiveIncludes.h>

namespace Settings
{
        const std::map<IResultFileType, std::string> IResultFileTypeMap = { { { IResultFileType::ResultFileType_undefined,"undefined" },
                                                                              { IResultFileType::ResultFileType_MATLAB,"MAT" },
                                                                              { IResultFileType::ResultFileType_HDF5 ,"HDF5" } } };

        std::string to_string(const IResultFileType& field)
        {
            return IResultFileTypeMap.at(field);
        }

        template<>
        IResultFileType from_string<IResultFileType>(const std::string &String)
        {
            for (auto it : IResultFileTypeMap)
                if (it.second == String)
                    return it.first;

            throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +String };
        }

#ifdef ARCHIVE_HAS_MATLAB
        template<>
        class ResultArchiveSelector<IResultFileType::ResultFileType_MATLAB>
        {
        public:
            using InputArchive = Archives::MatlabInputArchive;
            using OutputArchive = Archives::MatlabOutputArchive;
        };
#endif

#ifdef ARCHIVE_HAS_HDF5
        template<>
        class ResultArchiveSelector<IResultFileType::ResultFileType_HDF5>
        {
        public:
            using InputArchive = Archives::HDF5_InputArchive;
            using OutputArchive = Archives::HDF5_OutputArchive;
        };
#endif
}
