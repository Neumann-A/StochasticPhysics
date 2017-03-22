///-------------------------------------------------------------------------------------------------
// file:	ResultSettings.cpp
//
// summary:	Implements the result settings class
///-------------------------------------------------------------------------------------------------
#include "ResultSettings.h"

#include "MATLAB_Archive/Matlab_Archive.h"

namespace Settings
{
		std::string to_string(const IResultFileType& field)
		{
			return IResultFileTypeMap.at(field);
		};

		template<>
		IResultFileType from_string<IResultFileType>(const std::string &String)
		{
			for (auto it : IResultFileTypeMap)
				if (it.second == String)
					return it.first;

			throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +String };
		};


		template<>
		class ResultArchiveSelector<IResultFileType::ResultFileType_MATLAB>
		{
		public:
			using InputArchive = Archives::MatlabInputArchive;
			using OutputArchive = Archives::MatlabOutputArchive;
		};

}