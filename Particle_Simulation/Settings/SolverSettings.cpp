///-------------------------------------------------------------------------------------------------
// file:	SolverSettings.cpp
//
// summary:	Implements the solver settings class
///-------------------------------------------------------------------------------------------------
#include "SolverSettings.h"

namespace Settings
{
	const std::map<ISolver, std::string> ISolverMap = { { { ISolver::Solver_undefined,"undefined" },
													   { ISolver::Solver_EulerMaruyama,"EulerMaruyama" },
													   { ISolver::Solver_EulerMaruyamaNormalized,"EulerMaruyama_Normalized" },
													   { ISolver::Solver_Implicit_Midpoint,"Implicit_Midpoint" },
													   { ISolver::Solver_Implicit_Midpoint_GSL,"Implicit_Midpoint_GSL" },
													   { ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free,"Implicit_Midpoint_GSL_Derivative_Free" },
													   { ISolver::Solver_Millstein,"Millstein" },
													   { ISolver::Solver_Heun_Strong,"Heun_Strong" },
													   { ISolver::Solver_Heun_NotConsistent,"Heun_NotConsistent" },
													   { ISolver::Solver_WeakTest ,"WeakTest" },
													   { ISolver::Solver_ExplicitStrong1_0 ,"ExplicitStrong_1.0"} } };
#ifdef WITH_GSL_SOLVERS
	const std::map<gsl_solver_type, std::string> IGSLSolverMap = { {{ gsl_solver_type::undefined,"undefined" },
																	{ gsl_solver_type::newton,"newton" },
																	{ gsl_solver_type::gnewton,"gnewton" },
																	{ gsl_solver_type::hybridj,"hybridj" },
																	{ gsl_solver_type::hybridsj,"hybridsj" } } };

	const std::map<gsl_solver_type_derivative_free, std::string> IGSL2SolverMap = {{{ gsl_solver_type_derivative_free::undefined,"undefined" },
																					{ gsl_solver_type_derivative_free::hybrids,"hybrids" },
																					{ gsl_solver_type_derivative_free::hybrid,"hybrid" },
																					{ gsl_solver_type_derivative_free::dnewton,"dnewton" },
																					{ gsl_solver_type_derivative_free::broyden,"broyden" } } };
#endif
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
#ifdef WITH_GSL_SOLVERS
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
