///-------------------------------------------------------------------------------------------------
// file:	ProblemSettings.cpp
//
// summary:	Implements the problem settings class
///-------------------------------------------------------------------------------------------------
#include "ProblemSettings.h"

namespace Settings
{
	std::string to_string(const IProblem& field)
	{
		return IProblemMap.at(field);
	};

	template<>
	IProblem from_string<IProblem>(const std::string &String)
	{
		for (auto it : IProblemMap)
			if (it.second == String)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +String };
	};
}

