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
	ISolver from_string<ISolver>(const std::string &string)
	{
		for (auto it : ISolverMap)
			if (it.second == string)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +string };
	};

	/*****************************************************************************************************/
#ifdef USE_GSL_SOLVERS
	std::string to_string(const gsl_solver_type& field)
	{
		return IGSLSolverMap.at(field);
	};

	template<>
	gsl_solver_type from_string<gsl_solver_type>(const std::string &string)
	{
		for (auto it : IGSLSolverMap)
			if (it.second == string)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of ImplictGSLSolver unknown! " } +string };
	};

	/*****************************************************************************************************/

	std::string to_string(const gsl_solver_type_derivative_free& field)
	{
		return IGSL2SolverMap.at(field);
	};

	template<>
	gsl_solver_type_derivative_free from_string<gsl_solver_type_derivative_free>(const std::string &string)
	{
		for (auto it : IGSL2SolverMap)
			if (it.second == string)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of ImplictGSL2Solver unknown! " } +string };
	};
#endif
}
