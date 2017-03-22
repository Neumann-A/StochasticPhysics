///-------------------------------------------------------------------------------------------------
// file:	SolverSettings.cpp
//
// summary:	Implements the solver settings class
///-------------------------------------------------------------------------------------------------
#include "SolverSettings.h"

namespace Settings
{
	std::string to_string(const ISolver& field)
	{
		return ISolverMap.at(field);
	};

	template<>
	ISolver from_string<ISolver>(const std::string &String)
	{
		for (auto it : ISolverMap)
			if (it.second == String)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +String };
	};
}